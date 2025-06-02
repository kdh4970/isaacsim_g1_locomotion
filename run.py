# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from isaacsim import SimulationApp

DISP_FPS        = 1<<0
DISP_AXIS       = 1<<1
DISP_RESOLUTION = 1<<3
DISP_SKELEKETON   = 0<<9
DISP_MESH       = 0<<10
DISP_PROGRESS   = 0<<11
DISP_DEV_MEM    = 1<<13
DISP_HOST_MEM   = 1<<14

LAUNCH_CONFIG = {
    "headless":False,
    "fast_shutdown":True,
    "renderer":"RayTracedLighting",
    "multi_gpu":True,
    "window_width":1920,
    "window_height":1080,
    "display_options": DISP_FPS|DISP_RESOLUTION|DISP_MESH|DISP_DEV_MEM|DISP_HOST_MEM,
    }

app = SimulationApp(launch_config=LAUNCH_CONFIG)

import carb.tokens
import numpy as np
import omni.kit.commands
import omni.kit.test
from isaacsim.core.api import World
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.stage import create_new_stage
from g1 import G1FlatTerrainPolicy
from isaacsim.core.utils.stage import open_stage
World.clear_instance()
create_new_stage()
_physics_rate, _render_rate = 100, 30
_physics_dt = 1 / _physics_rate
_render_dt = 1 / _render_rate
_world = None

def init():
    global _world, tick, _base_command
    
    _base_command = [0, 0, 0]
    tick=0
    _g1 = None

    def on_physics_step(step_size):
        if _g1:
            _g1.forward(step_size, _base_command)

    _prim_path = "/World/g1"

    _g1 = G1FlatTerrainPolicy(prim_path=_prim_path, name="g1", position=np.array([0, 0, 0.76]))
    _world.reset()
    _g1.initialize()

    _world.add_physics_callback("physics_step", callback_fn=on_physics_step)

    print("[ Control Instructions ]")
    print(" ↑ : Move Forward")
    print(" ↓ : Stop")
    print(" ← : Rotate Left")
    print(" → : Rotate Right")
    print(" ESC : Reset Simulation")

create_new_stage()
# usd_path = "/home/do/Desktop/IsaacSIM-Robot-Simulation/usd_scenes/Collected_office/office.usd"
# open_stage(usd_path)
stage = omni.usd.get_context().get_stage()
app.update()



_world = World(stage_units_in_meters=1.0, physics_dt=_physics_dt, rendering_dt=_render_dt)
ground_prim = get_prim_at_path("/World/defaultGroundPlane")
# Create the physics ground plane if it hasn't been created
if not ground_prim.IsValid() or not ground_prim.IsActive():
    _world.scene.add_default_ground_plane()


_key_to_control = {
    "UP": [0.5,0,0],
    "DOWN": [0,0,0],
    "LEFT": [0,0,1],
    "RIGHT": [0,0,-1],
    "ZEROS": [0,0,0],
}
# Set up Keyboard
def _on_keyboard_event(event):
    global _base_command
    """Checks for a keyboard event and assign the corresponding command control depending on key pressed."""
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        # Arrow keys map to pre-defined command vectors to control navigation of robot
        if event.input.name in _key_to_control:
            _base_command = _key_to_control[event.input.name]
            print(f"Keyboard : {event.input.name}")
        if event.input.name == "ESCAPE":
            print("Resetting simulation.")
            _world.stop()
            init()
    # On key release, the robot stops moving
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        _base_command = _key_to_control["ZEROS"]

"""Sets up interface for keyboard input and registers the desired keys for control."""
_input = carb.input.acquire_input_interface()
_keyboard = omni.appwindow.get_default_app_window().get_keyboard()
_sub_keyboard = _input.subscribe_to_keyboard_events(_keyboard, _on_keyboard_event)

from time import perf_counter

_base_command = [0, 0, 0]
tick = 0
while app.is_running():
    app.update()
    while _world.is_playing():
        now = perf_counter()
        if tick==0:
            next_physics_time = now
            next_render_time = now
            init()
        
        while now > next_physics_time:
            _world.step(render = False)
            next_physics_time += _physics_dt

        if now > next_render_time:
            _world.render()
            next_render_time += _render_dt

        
        tick += 1
app.close()