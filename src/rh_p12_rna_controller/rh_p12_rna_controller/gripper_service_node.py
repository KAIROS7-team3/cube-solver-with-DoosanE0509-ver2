#!/usr/bin/env python3
from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from rh_p12_rna_controller.gripper_node import GripperNode, ModbusRTU
from cube_interfaces.action import SafeGrasp
from cube_interfaces.srv import GetState, SetPosition
from cube_interfaces.msg import GripperState


class GripperServiceNode(GripperNode):
    """Owner node that keeps a single TCP bridge and exposes stable APIs."""

    def __init__(self):
        super().__init__()

        # Services
        self._srv_get_state = self.create_service(GetState, "/gripper/get_state", self._on_get_state)
        self._srv_set_position = self.create_service(
            SetPosition, "/gripper/set_position", self._on_set_position
        )

        # Safe grasp action
        self._safe_grasp_action = ActionServer(
            self,
            SafeGrasp,
            "/gripper/safe_grasp",
            execute_callback=self._execute_safe_grasp,
            goal_callback=self._goal_safe_grasp,
            cancel_callback=self._cancel_safe_grasp,
        )

        self.get_logger().info("gripper_service_node ready: /gripper/get_state, /gripper/set_position, /gripper/safe_grasp")

    def _goal_safe_grasp(self, goal_request):
        # reject if already executing any action
        if getattr(self, "_executing", False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_safe_grasp(self, goal_handle):
        return CancelResponse.ACCEPT

    def _on_get_state(self, request, response):
        msg = GripperState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = int(self._current_hz_pos)
        msg.current = int(self._current_hz_cur)
        msg.goal_position = int(getattr(self, "_last_goal_pos", 0) or 0)
        msg.goal_current = int(getattr(self, "_last_goal_cur", 0) or 0)
        msg.status_text = "ok" if self._socket_active else "tcp_offline"
        response.state = msg
        return response

    def _on_set_position(self, request: SetPosition.Request, response: SetPosition.Response):
        # Best-effort direct set; does not run full "gripped" logic.
        try:
            pos = int(request.position)
            cur = int(request.current)
            timeout = float(request.timeout_sec) if request.timeout_sec > 0 else 5.0
            self._last_goal_pos = pos
            self._last_goal_cur = cur

            if self._cmd_transport == "drl":
                # Reuse DRL move+poll helper for robust completion.
                code = self._build_drl_move_and_poll_for_service(pos, cur, user_timeout=timeout)
                ok = self._call_drl(code, timeout_sec=timeout + 5.0)
                # DRL ok가 false여도 실측 위치가 도달이면 success.
                position_reached = abs(int(self._current_hz_pos) - pos) <= int(self._done_tol)
                response.success = bool(ok or position_reached)
                if response.success:
                    response.message = "ok(drl)" if ok else "position_reached(drl_warn)"
                else:
                    response.message = "drl failed and position not reached"
            else:
                if not self._socket_active:
                    response.success = False
                    response.message = "tcp offline"
                    return response
                ok, err = self._send_cmd_and_wait_ack(
                    [
                        ModbusRTU.fc06(self._slave_id, self._goal_cur_reg, cur),
                    ],
                    timeout_sec=timeout,
                )
                if not ok:
                    response.success = False
                    response.message = f"cur ack failed: {err}"
                    return response

                # position write (default fc16)
                goal_val = int(round(float(pos) * (self._goal_pos_scale if self._goal_pos_scale else 1.0)))
                pkt = ModbusRTU.fc16(self._slave_id, self._goal_pos_reg, 2, [goal_val & 0xFFFF, 0])
                ok2, err2 = self._send_cmd_and_wait_ack([pkt], timeout_sec=timeout)
                response.success = bool(ok2)
                response.message = "ok(tcp)" if ok2 else f"pos ack failed: {err2}"

            response.final_position = int(self._current_hz_pos)
            response.final_current = int(self._current_hz_cur)
            return response
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.final_position = int(self._current_hz_pos)
            response.final_current = int(self._current_hz_cur)
            return response

    def _build_drl_move_and_poll_for_service(self, pos: int, cur: int, user_timeout: float = 8.0) -> str:
        # Import from base module-level helper through instance globals.
        # We just call the existing function defined in gripper_node.py namespace.
        from rh_p12_rna_controller.gripper_node import build_drl_move_and_poll  # local import

        # Each DRL polling iteration takes ~0.85s (read_cur + read_pos + wait(0.15)).
        # Size max_loops to roughly match user timeout, with a small safety margin,
        # and clamp to a sane range so a tiny timeout does not yield zero loops.
        per_loop_sec = 0.85
        loops = int(max(1.0, float(user_timeout)) / per_loop_sec) + 4
        loops = max(8, min(80, loops))

        return build_drl_move_and_poll(
            slave_id=self._slave_id,
            target_pulse=int(pos),
            target_current=int(cur),
            grip_current_threshold=int(self._grip_threshold),
            pos_tolerance=int(self._done_tol),
            max_loops=loops,
        )

    def _execute_safe_grasp(self, goal_handle):
        self._executing = True
        try:
            req = goal_handle.request
            target = int(req.target_position)
            gcur = int(req.goal_current)
            thr = int(req.current_threshold)
            timeout = float(req.timeout_sec) if req.timeout_sec > 0 else 8.0

            self._last_goal_pos = target
            self._last_goal_cur = gcur

            # Send command using existing TCP path (fast), then monitor state.
            ok, msg = True, ""
            if self._cmd_transport == "tcp":
                ok1, err1 = self._send_cmd_and_wait_ack(
                    [ModbusRTU.fc06(self._slave_id, self._goal_cur_reg, gcur)],
                    timeout_sec=min(3.0, timeout),
                )
                if not ok1:
                    ok = False
                    msg = f"cur ack failed: {err1}"
                else:
                    goal_val = int(round(float(target) * (self._goal_pos_scale if self._goal_pos_scale else 1.0)))
                    pkt = ModbusRTU.fc16(self._slave_id, self._goal_pos_reg, 2, [goal_val & 0xFFFF, 0])
                    ok2, err2 = self._send_cmd_and_wait_ack([pkt], timeout_sec=min(3.0, timeout))
                    if not ok2:
                        ok = False
                        msg = f"pos ack failed: {err2}"
            else:
                # drl mode: use DRL move+poll to completion.
                # _call_drl 대기 시간을 user timeout 기반으로 제한 — 그리퍼가 1~2s에 끝나는데
                # 60s 천장이 잡혀 있으면 DRL polling이 이상하게 늘어졌을 때 액션 응답이 끔찍하게 늦음.
                # 어차피 success는 실측 위치(position_reached) 기반으로 판정하므로 DRL이 일찍 끊겨도 무방.
                code = self._build_drl_move_and_poll_for_service(target, gcur, user_timeout=timeout)
                ok = bool(self._call_drl(code, timeout_sec=timeout + 5.0))
                msg = "ok(drl)" if ok else "drl failed/slow"

            start = time.time()
            grasped = False
            while (time.time() - start) < timeout and rclpy.ok():
                fb = SafeGrasp.Feedback()
                fb.phase = "moving"
                fb.current_position = int(self._current_hz_pos)
                fb.current = int(self._current_hz_cur)
                fb.progress = min(1.0, float(time.time() - start) / max(0.1, timeout))
                goal_handle.publish_feedback(fb)

                if abs(int(self._current_hz_cur)) >= thr:
                    grasped = True
                    break
                # if reached position sufficiently, also stop loop
                if abs(int(self._current_hz_pos) - target) <= int(self._done_tol):
                    break
                time.sleep(0.05)

            # 권위 기준: 실제 위치/전류. DRL 호출(ok)은 부가 정보로만 사용.
            # → 그리퍼가 물리적으로 목표 위치에 도달했거나 grasped(전류 임계 초과)면 success.
            #   DRL 폴링이 readback 일시 실패로 false를 반환해도, 실측 위치가 도달이면 성공 처리.
            position_reached = abs(int(self._current_hz_pos) - target) <= int(self._done_tol)
            result = SafeGrasp.Result()
            result.success = bool(grasped or position_reached)
            result.grasped = bool(grasped)
            result.final_position = int(self._current_hz_pos)
            result.final_current = int(self._current_hz_cur)
            if result.success:
                if grasped:
                    result.message = "grasped"
                elif not ok:
                    # DRL은 실패라고 했지만 위치는 도달 — readback 오류 등 무시
                    result.message = f"position_reached (drl_warn: {msg})"
                else:
                    result.message = "position_reached"
            else:
                result.message = msg if msg else "neither grasped nor position_reached"

            if result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        finally:
            self._executing = False


def main(args=None):
    rclpy.init(args=args)
    node = GripperServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

