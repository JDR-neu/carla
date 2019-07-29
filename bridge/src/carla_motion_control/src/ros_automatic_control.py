#!/usr/bin/env python

from __future__ import print_function

import collections
import datetime
import glob
import math
import os
import random
import re
import sys
import weakref
import time
import numpy as np
import wiringpi

import pygame
from pygame.locals import KMOD_CTRL
from pygame.locals import KMOD_SHIFT
from pygame.locals import K_0
from pygame.locals import K_9
from pygame.locals import K_BACKQUOTE
from pygame.locals import K_BACKSPACE
from pygame.locals import K_COMMA
from pygame.locals import K_DOWN
from pygame.locals import K_ESCAPE
from pygame.locals import K_F1
from pygame.locals import K_LEFT
from pygame.locals import K_PERIOD
from pygame.locals import K_RIGHT
from pygame.locals import K_SLASH
from pygame.locals import K_SPACE
from pygame.locals import K_TAB
from pygame.locals import K_UP
from pygame.locals import K_a
from pygame.locals import K_c
from pygame.locals import K_d
from pygame.locals import K_h
from pygame.locals import K_m
from pygame.locals import K_p
from pygame.locals import K_q
from pygame.locals import K_r
from pygame.locals import K_s
from pygame.locals import K_w
from pygame.locals import K_MINUS
from pygame.locals import K_EQUALS


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================
carla_module = glob.glob('/home/goujs/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
    sys.version_info.major,
    sys.version_info.minor,
    'win-amd64' if os.name == 'nt' else 'linux-x86_64'))
print(carla_module)
sys.path.append(carla_module[0])
try:
    sys.path.append(glob.glob('PythonAPI')[0])
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

sys.path.append("/home/goujs/carla/PythonAPI/examples")
sys.path.append("/home/goujs/carla/PythonAPI/carla")

from carla import Vector3D, Rotation, Location, Transform
from agents.navigation.basic_agent import BasicAgent

# ==============================================================================
# -- ROS and SYS_ROS --------------------------------------------
# ==============================================================================

import rospy
from trajectory_pub.msg import PathPoint, TrajectoryPoint, DiscretizedTrajectory

def callback(traj):
    global g_world
    rospy.loginfo("receive a new trajectory")
    if isinstance(g_world.player, carla.Vehicle):
        if traj is not None:
            g_world.player.set_autopilot(True)
            for pt in traj.discretized_trajectory_:
                g_world.player.set_fixed_route_one_point(pt.path_point_.x_, pt.path_point_.y_, 120.0)
                g_world.player.set_speed_limit(pt.v_, True)
                time.sleep(0.001)
                # wiringpi.delayMicroseconds(10)


def listener():
    rospy.init_node('carla_motion_control', anonymous=False)
    rospy.Subscriber("trajectory", DiscretizedTrajectory, callback)
    # rospy.spin()


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================

class World(object):
    def __init__(self, carla_world, actor_filter):
        global g_hud
        self.world = carla_world
        self.map = self.world.get_map()
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = actor_filter
        self.restart()
        self.world.on_tick(g_hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0

    def restart(self):
        global g_spawn_point
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        # blueprint = random.choice(self.world.get_blueprint_library().filter('vehicle.*')) #self._actor_filter))
        blueprint = random.choice(self.world.get_blueprint_library().filter('vehicle.tesla.model3'))
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)

        if self.player is not None:
            # info = "1 -- g_spawn_point.location = (%.3f, %.3f, %.3f), g_spawn_point.rotation = (%.3f, %.3f, %.3f)" % \
            #        (g_spawn_point.location.x, g_spawn_point.location.y, g_spawn_point.location.z,
            #         g_spawn_point.rotation.roll, g_spawn_point.rotation.pitch, g_spawn_point.rotation.yaw)
            # print(info)
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, g_spawn_point)
        while self.player is None:
            # info = "2 -- g_spawn_point.location = (%.3f, %.3f, %.3f), g_spawn_point.rotation = (%.3f, %.3f, %.3f)" % \
            #        (g_spawn_point.location.x, g_spawn_point.location.y, g_spawn_point.location.z,
            #         g_spawn_point.rotation.roll, g_spawn_point.rotation.pitch, g_spawn_point.rotation.yaw)
            # print(info)
            self.player = self.world.try_spawn_actor(blueprint, g_spawn_point)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, g_hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, g_hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, g_hud)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        g_hud.notification(actor_type)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        g_hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        g_hud.tick(clock)

    def render(self, display):
        self.camera_manager.render(display)
        g_hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        actors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================

class KeyboardControl(object):
    def __init__(self):
        global g_world, g_spawn_point
        self._autopilot_enabled = True
        g_world.player.set_autopilot(True)
        if isinstance(g_world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
        else:
            print("Error: no vehicle instance")

        self._steer_cache = 0.0
        g_hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        g_world.player.set_autopilot(True)

        self.agent = BasicAgent(g_world.player)
        self.agent.set_destination((g_spawn_point.location.x,
                                    g_spawn_point.location.y,
                                    g_spawn_point.location.z))


    def parse_events(self, clock):
        global g_world, g_client
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("pygame.QUIT")
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    print("quit")
                    return True
                elif event.key == K_BACKSPACE:
                    print("K_BACKSPACE")
                    g_world.restart()
                elif event.key == K_F1:
                    print("K_F1")
                    g_hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    print("K_h K_SLASH")
                    g_hud.help.toggle()
                elif event.key == K_TAB:
                    print("K_TAB")
                    g_world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    print("K_c KMOD_SHIFT")
                    g_world.next_weather(reverse=True)
                elif event.key == K_c:
                    print("K_c")
                    g_world.next_weather()
                elif event.key == K_BACKQUOTE:
                    print("K_BACKQUOTE")
                    g_world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    print("0-9")
                    g_world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    print("K_r not KMOD_CTRL")
                    g_world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    print("K_r")
                    if (g_world.recording_enabled):
                        g_client.stop_recorder()
                        g_world.recording_enabled = False
                        g_hud.notification("Recorder is OFF")
                    else:
                        g_client.start_recorder("manual_recording.rec")
                        g_world.recording_enabled = True
                        g_hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    print("K_p")
                    # stop recorder
                    g_client.stop_recorder()
                    g_world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    currentIndex = g_world.camera_manager.index
                    g_world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    g_world.player.set_autopilot(self._autopilot_enabled)
                    g_hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    g_client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    g_world.camera_manager.set_sensor(currentIndex)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    print("K_MINUS")
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        g_world.recording_start -= 10
                    else:
                        g_world.recording_start -= 1
                    g_hud.notification("Recording start time is %d" % (g_world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    print("K_EQUALS")
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        g_world.recording_start += 10
                    else:
                        g_world.recording_start += 1
                    g_hud.notification("Recording start time is %d" % (g_world.recording_start))
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        print("K_q")
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        print("K_m")
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = g_world.player.get_control().gear
                        g_hud.notification('%s Transmission' % (
                            'Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        print("K_COMMA")
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        print("K_PERIOD")
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p and not (pygame.key.get_mods() & KMOD_CTRL):
                        print("K_p not KMOD_CTRL")
                        self._autopilot_enabled = not self._autopilot_enabled
                        g_world.player.set_autopilot(self._autopilot_enabled)
                        g_hud.notification(
                            'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                keys = pygame.key.get_pressed()
                if sum(keys) > 0:
                    self._parse_vehicle_keys(keys, clock.get_time())
                    self._control.reverse = self._control.gear < 0
                    g_world.player.apply_control(self._control)
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
                g_world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_walker_keys(self, keys, milliseconds):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================

class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        fonts = [x for x in pygame.font.get_fonts() if 'mono' in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))

        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, clock):
        global g_world
        self._notifications.tick(g_world, clock)
        if not self._show_info:
            return
        t = g_world.player.get_transform()
        v = g_world.player.get_velocity()
        c = g_world.player.get_control()
        acce = g_world.player.get_acceleration()

        heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
        heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
        heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
        heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
        colhist = g_world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = g_world.world.get_actors().filter('vehicle.*')

        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(g_world.player, truncate=20),
            'Map:     % 20s' % g_world.map.name,
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 6.2f km/h' % (3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)),
            'Speed-x: % 6.2f km/h' % (3.6 * v.x),
            'Speed-y: % 6.2f km/h' % (3.6 * v.y),
            'Speed-z: % 6.2f km/h' % (3.6 * v.z),
            '',
            'Acceleration:   % 6.2f m/s2' % (math.sqrt(acce.x ** 2 + acce.y ** 2 + acce.z ** 2)),
            'Acceleration-x: %6.2f m/s2' % (acce.x),
            'Acceleration-y: %6.2f m/s2' % (acce.y),
            'Acceleration-z: %6.2f m/s2' % (acce.z),
            '',

            'Throttle:% 6.2f' % (c.throttle),
            'Steer:   % 6.2f' % (c.steer),
            'Brake:   % 6.2f' % (c.brake),
            '',

            u'Heading:% 6.2f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (g_world.gnss_sensor.lat, g_world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']

            def distance(l): return math.sqrt(
                (l.x - t.location.x) ** 2 + (l.y - t.location.y) ** 2 + (l.z - t.location.z) ** 2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != g_world.player.id]
            for d, vehicle in sorted(vehicles):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================

class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================

class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        g_hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        g_hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)

# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================

class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self._parent = parent_actor
        g_hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        g_hud.notification('Crossed line %s' % ' and '.join(text))

# ==============================================================================
# -- GnssSensor --------------------------------------------------------
# ==============================================================================

class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)),
                                        attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================

class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        self._camera_transforms = [
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            carla.Transform(carla.Location(x=1.6, z=1.7))]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
            ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
            ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
            ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
             'Camera Semantic Segmentation (CityScapes Palette)'],
            ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '5000')
            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.sensor.set_transform(self._camera_transforms[self.transform_index])

    def set_sensor(self, index, notify=True):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None \
            else self.sensors[index][0] != self.sensors[self.index][0]
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index],
                attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            g_hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        g_hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 3), 3))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(g_hud.dim) / 100.0
            lidar_data += (0.5 * g_hud.dim[0], 0.5 * g_hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (g_hud.dim[0], g_hud.dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)

# ==============================================================================
# -- game_loop() ---------------------------------------------------------
# ==============================================================================

class PygamePlayer(object):
    init_flag = False

    def __init__(self):
        global g_world
        if PygamePlayer.init_flag:
            return
        PygamePlayer.init_flag = True
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(
            (1280, 720),
            pygame.HWSURFACE | pygame.DOUBLEBUF)


    def game_loop(self):
        global g_world, g_client, g_controller
        print("game_loop")
        clock = pygame.time.Clock()

        while True:
            if g_controller.parse_events(clock):
                print("should exit")
                rospy.signal_shutdown("closed!")
                return

            # as soon as the server is ready continue!
            g_world.world.wait_for_tick(10.0)
            g_world.tick(clock)
            g_world.render(self.display)
            pygame.display.flip()

# Location(x, y, z)  Rotation(pitch, yaw, roll)
spawn_point_straight = Transform(Location(8.90147, 96.35236, 1.20), Rotation(0, 90, 0))
spawn_point_long = Transform(Location(-341.555, 26.4891, 1.20), Rotation(0, 0, 0))
g_spawn_point = spawn_point_long

g_client = carla.Client('127.0.0.1', 2000)
g_client.set_timeout(4.0)
g_game = PygamePlayer()
g_hud = HUD(1280, 720)
g_world = World(g_client.get_world(), 'vehicle.*')
g_controller = KeyboardControl()



if __name__ == '__main__':
    global g_game
    listener()
    g_game.game_loop()

