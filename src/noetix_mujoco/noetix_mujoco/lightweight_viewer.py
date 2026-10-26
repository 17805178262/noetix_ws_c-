"""Small GLFW + native MuJoCo renderer, without MuJoCo's inspector/UI panels.

The window is owned by the calling (main) thread. It uses its own MjData
instance; the physics thread never exposes its live MjData to the renderer.
GUI events only enqueue requests, never mutate the physics model or state.
"""
from __future__ import annotations

import time

import numpy as np


class LightweightViewer:
    """Render scene snapshots at the requested FPS and handle camera/keys."""

    def __init__(self, mj, model, *, get_snapshot, on_action, config, robot_name, spawn_z):
        import glfw

        self.mj = mj
        self.glfw = glfw
        self.model = model
        self.get_snapshot = get_snapshot
        self.on_action = on_action
        self.config = config
        self.fps = float(config['gui_fps'])
        self.robot_name = robot_name
        self.spawn_z = float(spawn_z)
        self.window = None
        self.scene = None
        self.context = None
        self.render_data = mj.MjData(model)
        self.camera = mj.MjvCamera()
        self.camera.type = mj.mjtCamera.mjCAMERA_FREE
        self.camera.lookat[:] = (0.0, 0.0, max(self.spawn_z * 0.7, 0.8))
        self.camera.distance = max(2.8, float(model.stat.extent) * 1.8)
        self.camera.azimuth = 135.0
        self.camera.elevation = -18.0
        self.vis = mj.MjvOption()
        self._mouse_x = 0.0
        self._mouse_y = 0.0
        self._prev_snapshot_time = None
        self._frame_count = 0

    def open(self):
        glfw = self.glfw
        if not glfw.init():
            raise RuntimeError('GLFW initialization failed (check DISPLAY/OpenGL driver)')
        try:
            glfw.window_hint(glfw.SAMPLES, 0)  # minimum multisampling cost
            glfw.window_hint(glfw.RESIZABLE, glfw.TRUE)
            glfw.window_hint(glfw.VISIBLE, glfw.TRUE)
            self.window = glfw.create_window(
                1152, 720,
                f'Noetix MuJoCo - {self.robot_name} | H hoist | R reset | Space pause',
                None, None,
            )
            if self.window is None:
                raise RuntimeError('Could not create GLFW window (check GPU/OpenGL driver)')
            glfw.make_context_current(self.window)
            # Our loop limits actual GUI FPS; do not block on display vsync.
            glfw.swap_interval(0)
            self.scene = self.mj.MjvScene(self.model, maxgeom=max(1000, self.model.ngeom * 3))
            self.context = self.mj.MjrContext(self.model, self.mj.mjtFontScale.mjFONTSCALE_100)
            self.mj.mjr_setBuffer(self.mj.mjtFramebuffer.mjFB_WINDOW, self.context)
            self.scene.flags[self.mj.mjtRndFlag.mjRND_SHADOW] = bool(
                self.config['render']['shadows'])
            self.scene.flags[self.mj.mjtRndFlag.mjRND_REFLECTION] = bool(
                self.config['render']['reflections'])
            glfw.set_key_callback(self.window, self._on_key)
            glfw.set_mouse_button_callback(self.window, self._on_mouse_button)
            glfw.set_cursor_pos_callback(self.window, self._on_cursor_pos)
            glfw.set_scroll_callback(self.window, self._on_scroll)
        except BaseException:
            self.close()
            raise

    def _on_key(self, window, key, _scan, action, mods):
        """H toggles hoist; Ctrl+R has no effect in the MuJoCo window."""
        glfw = self.glfw
        if action != glfw.PRESS:
            return
        if key == glfw.KEY_H:
            self.on_action('hoist_toggle')
        elif key == glfw.KEY_R:
            control_down = bool(mods & glfw.MOD_CONTROL) or (
                glfw.get_key(window, glfw.KEY_LEFT_CONTROL) == glfw.PRESS
                or glfw.get_key(window, glfw.KEY_RIGHT_CONTROL) == glfw.PRESS)
            if not control_down:
                self.on_action('reset')
        elif key == glfw.KEY_SPACE:
            self.on_action('pause_toggle')
        elif key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(window, True)

    def _on_mouse_button(self, window, _button, _action, _mods):
        self._mouse_x, self._mouse_y = self.glfw.get_cursor_pos(window)

    def _on_cursor_pos(self, window, x, y):
        glfw = self.glfw
        left = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
        middle = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS
        right = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
        dx, dy = x - self._mouse_x, y - self._mouse_y
        self._mouse_x, self._mouse_y = x, y
        if not (left or middle or right):
            return
        _, height = glfw.get_window_size(window)
        if height < 1:
            return
        shift = (glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
                 or glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)
        if right:
            mode = (self.mj.mjtMouse.mjMOUSE_MOVE_H if shift
                    else self.mj.mjtMouse.mjMOUSE_MOVE_V)
        elif left:
            mode = (self.mj.mjtMouse.mjMOUSE_ROTATE_H if shift
                    else self.mj.mjtMouse.mjMOUSE_ROTATE_V)
        else:
            mode = self.mj.mjtMouse.mjMOUSE_ZOOM
        self.mj.mjv_moveCamera(self.model, mode, dx / height, dy / height,
                              self.scene, self.camera)

    def _on_scroll(self, _window, _dx, dy):
        self.mj.mjv_moveCamera(self.model, self.mj.mjtMouse.mjMOUSE_ZOOM,
                              0.0, -0.05 * dy, self.scene, self.camera)

    def _render(self):
        snapshot = self.get_snapshot()
        if snapshot is None:
            return
        (qpos, qvel, mocap_pos, mocap_quat, eq_active, t) = snapshot
        np.copyto(self.render_data.qpos, qpos)
        np.copyto(self.render_data.qvel, qvel)
        np.copyto(self.render_data.mocap_pos, mocap_pos)
        np.copyto(self.render_data.mocap_quat, mocap_quat)
        np.copyto(self.render_data.eq_active, eq_active)
        self.render_data.time = t
        self.mj.mj_forward(self.model, self.render_data)
        self.mj.mjv_updateScene(self.model, self.render_data, self.vis, None,
                                self.camera, self.mj.mjtCatBit.mjCAT_ALL, self.scene)
        width, height = self.glfw.get_framebuffer_size(self.window)
        if width <= 0 or height <= 0:  # minimized
            return
        self.mj.mjr_render(self.mj.MjrRect(0, 0, width, height), self.scene, self.context)
        self.glfw.swap_buffers(self.window)
        self._frame_count += 1

    def run(self, keep_running):
        frame_dt = 1.0 / self.fps
        next_frame = time.monotonic()
        while keep_running() and not self.glfw.window_should_close(self.window):
            # GLFW input remains responsive between rendered frames.
            self.glfw.poll_events()
            now = time.monotonic()
            if now >= next_frame:
                self._render()
                next_frame += frame_dt
                if next_frame < time.monotonic() - frame_dt:
                    next_frame = time.monotonic() + frame_dt
            else:
                time.sleep(min(next_frame - now, 0.005))

    def close(self):
        if self.window is None:
            return
        self.glfw.make_context_current(self.window)
        # Context is associated with GLFW's current OpenGL context.
        self.context = None
        self.scene = None
        self.glfw.destroy_window(self.window)
        self.window = None
        self.glfw.terminate()
