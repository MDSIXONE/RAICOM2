import asyncio
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0,str(Path(__file__).resolve().parent))
import server
from motion_limits import enforce_axes, MIN_VX, MIN_VY_SETTING, MIN_YAW_SETTING
from line_follow import validate_line_config, load_line_config


class MotionFloorTest(unittest.TestCase):
    def test_all_nonzero_axes_are_outside_directional_deadzone(self):
        for sign in (-1,1):
            for value in (-20,-8,-1,-0.0001,0,0.0001,1,8,20):
                vx,vy,yaw=enforce_axes(value,value,value,sign)
                if value==0:
                    self.assertEqual((vx,vy,yaw),(0,0,0))
                else:
                    self.assertGreaterEqual(abs(vx),3)
                    self.assertGreaterEqual(abs(vy),8 if vy*sign>0 else 2.5)
                    self.assertGreaterEqual(abs(yaw),16 if yaw>0 else 14)

    def test_actual_robot_client_guards_manual_auto_line_and_wall_outputs(self):
        state=server.SharedState()
        client=server.RobotClient()
        self.addCleanup(client._udp.close)
        with patch.object(server,'STATE',state),patch.object(client,'_send_gamepad_udp') as send:
            for mode in ('manual','auto','line','wall'):
                client.axes(0.01,0.01,-0.01,mode)
                self.assertEqual(send.call_args.args,(3.0,8.0,-14.0))
                self.assertEqual(state.robot['vx'],3)
                self.assertEqual(state.robot['vy'],8)
                self.assertEqual(state.robot['yaw_rate'],-14)
            client.axes(0,0,0)
            self.assertEqual(send.call_args.args,(0,0,0))

    def test_backend_rejects_subminimum_speed_configuration_in_both_legs(self):
        for key,minimum in (
            ('cruise_vx',3),('avoid_vx',3),('max_vy',8),
            ('wall_forward_vx',3),('wall_min_vy',8),('wall_max_vy',8),('wall_max_yaw_rate',16),
            ('wall_second_forward_vx',3),('wall_second_min_vy',8),('wall_second_max_vy',8),('wall_second_max_yaw_rate',16),
            ('wall_final_forward_vx',3),('wall_final_min_vy',8),('wall_final_max_vy',8),('wall_final_max_yaw_rate',16),
        ):
            with self.subTest(key=key):
                config={**server.DEFAULT_CONFIG,key:minimum-0.01}
                self.assertIsNotNone(server.validate_config(config))

    def test_manual_endpoint_rejects_deadzone_nan_and_accepts_stop(self):
        async def invoke(body):
            request=Mock()
            async def json_body():return body
            request.json=json_body
            return await server.api_manual(request)
        state=server.SharedState()
        robot=Mock()
        with patch.object(server,'STATE',state),patch.object(server,'ROBOT',robot):
            for body in ({'vx':2.9},{'vy':7.9},{'vy':-2.4},{'yaw':15.9},{'yaw':-13.9},{'vx':float('nan')}):
                self.assertEqual(asyncio.run(invoke(body)).status,400)
            robot.axes.assert_not_called()
            self.assertEqual(asyncio.run(invoke({'vx':0,'vy':0,'yaw':0})).status,200)
            robot.axes.assert_called_once_with(0,0,0,'manual')

    def test_old_low_wall_and_line_config_are_upgraded_explicitly(self):
        old={**server.DEFAULT_CONFIG,'wall_max_yaw_rate':12,'wall_final_max_yaw_rate':12,'wall_min_vy':4.5,'wall_max_vy':7,'wall_final_min_vy':4.5,'wall_final_max_vy':7}
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.json'
            path.write_text(json.dumps(old),encoding='utf-8')
            with patch.object(server,'CONFIG_FILE',path):
                config=server.load_saved_config()
            self.assertEqual(config['wall_max_yaw_rate'],16)
            self.assertEqual(config['wall_min_vy'],8)
            self.assertEqual(config['wall_final_max_vy'],8)
            path.write_text(json.dumps({'turn_speed_min':12,'turn_speed_max':15}),encoding='utf-8')
            line=load_line_config(path)
            self.assertEqual(line['turn_speed_min'],16)
            self.assertEqual(line['turn_speed_max'],16)

    def test_line_parameters_cannot_reintroduce_deadzone(self):
        for change in ({'straight_speed':2.9},{'turn_speed_min':15},{'probe_turn_speed':-13},{'probe_turn_speed':15},{'surge_turn_speed':-1}):
            with self.subTest(change=change):
                with self.assertRaises(ValueError):validate_line_config(change)

    def test_slowdown_and_tiny_gains_cannot_round_to_a_dead_command(self):
        cfg={**server.DEFAULT_CONFIG,'wall_yaw_kp':0.1,'wall_lateral_kp':1.0}
        vx,vy,yaw,*_=server.compute_wall_command(0.30000001,0.35,14,10,cfg)
        self.assertEqual(vx,3)
        self.assertEqual(vy,8)
        self.assertEqual(yaw,-14)
        vx,vy,yaw,*_=server.compute_wall_command(0.30,0.30,10,10,cfg)
        self.assertEqual((vx,vy,yaw),(0,0,0))
