import re

class MotionInterface:
    def __init__(self):
        self.action_meta = {
            0: {
                "name": "急停",
                "mode": 0,
                "gait_id": 0,
                "allowed_switch": ["RECOVERY_STAND=12", "POSE_CTRL=21", "PUREDAMPER=7", "MOTOR_CTRL=15", "MOTION=62", "OFF=0"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (0, 0),
                        "default": 0,
                        "desc": "时间[ms]，固定为0"
                    }
                },
                "remark": "紧急停止所有动作"
            },
            111: {
                "name": "站立",
                "mode": 12,
                "gait_id": 0,
                "allowed_switch": ["全部"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (0, 6000),
                        "default": 0,
                        "desc": "时间[ms]，0=持续动作；0-6000ms"
                    }
                },
                "remark": "进入时若发现倾倒，有自动翻身机制"
            },
            201: {
                "name": "姿态展示",
                "mode": 3,
                "gait_id": 0,
                "allowed_switch": ["LOCOMOTION=11", "PUREDAMPER=7", "POSE_CTRL=21", "RECOVERY_STAND=12", "OFF=0", "QP_STAND=3"],
                "params": {
                    "rpy_des": {
                        "type": list,
                        "range": [(-0.52, 0.52), (-0.25, 0.25), (-0.65, 0.65)],
                        "default": [0.0, 0.0, 0.0],
                        "desc": "机身期望姿态角[rad] [横滚,俯仰,偏航]"
                    },
                    "pos_des": {
                        "type": list,
                        "range": [(-0.32, 0.32), (-0.32, 0.32), (-0.32, 0.32)],
                        "default": [0.0, 0.0, 0.25],
                        "desc": "机身期望位置[m] [前后,左右,上下]（相对,绝对,绝对）"
                    },
                    "contact": {
                        "type": str,
                        "range": r"^[01]{4}$",
                        "default": "1111",
                        "desc": "四条腿着地状态，如0b00011111表示四条腿着地"
                    },
                    "duration": {
                        "type": int,
                        "range": (0, None),
                        "default": 0,
                        "desc": "时间[ms]，0=持续动作；>0=指定时间"
                    }
                },
                "remark": "展示机身姿态，支持腿着地状态配置"
            },
            102: {
                "name": "高阻趴卧下",
                "mode": 7,
                "gait_id": 0,
                "allowed_switch": ["OFF=0", "RECOVERY_STAND=12"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (0, None),
                        "default": 0,
                        "desc": "时间[ms]，0或>3000ms（少于实际值时会吞掉后续动作）"
                    }
                },
                "remark": "高阻保护动作，有趴下的效果，但趴下位置不固定"
            },
            101: {
                "name": "受控趴下",
                "mode": 7,
                "gait_id": 1,
                "allowed_switch": ["RECOVERY_STAND=12", "PUREDAMPER=7"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (0, None),
                        "default": 0,
                        "desc": "时间[ms]，0或>3000ms（少于实际值时会吞掉后续动作）"
                    }
                },
                "remark": "受控的插值趴下，可以趴到一个固定的位置"
            },
            112: {
                "name": "零速行走下的站立",
                "mode": 11,
                "gait_id": 1,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3", "LOCOMOTION=11"],
                "params": {
                    "vel_des": {
                        "type": list,
                        "range": [(0, 0), (0, 0), (0, 0)],
                        "default": [0.0, 0.0, 0.0],
                        "desc": "期望速度[m/s] [x,y,yaw]"
                    },
                    "duration": {
                        "type": int,
                        "range": (0, None),
                        "default": 0,
                        "desc": "时间[ms]，0=持续动作"
                    },
                    "rpy_des": {
                        "type": list,
                        "range": [(-0.25, 0.3), (0, 0), (0, 0)],
                        "default": [0.0, 0.0, 0.0],
                        "desc": "机身俯仰角[rad]"
                    },
                    "pos_des": {
                        "type": list,
                        "range": [(0, 0), (0, 0), (0.2, 0.28)],
                        "default": [0.0, 0.0, 0.25],
                        "desc": "机身高度[m]"
                    },
                    "step_height": {
                        "type": list,
                        "range": [(0, 0.06), (0, 0.06)],
                        "default": [0.06, 0.06],
                        "desc": "抬腿高度[m]"
                    }
                },
                "remark": "零速行走模式下的站立动作"
            },
            302: {
                "name": "四腿蹦跳",
                "mode": 11,
                "gait_id": 2,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3", "LOCOMOTION=11"],
                "params": {
                    "vel_des": {
                        "type": list,
                        "range": [(-0.10, 0.15), (-0.15, 0.50), (-0.20, 0.15)],
                        "default": [0.0, 0.0, 0.0],
                        "desc": "期望速度[m/s] [x,y,yaw]"
                    },
                    "duration": {
                        "type": int,
                        "range": (0, None),
                        "default": 0,
                        "desc": "时间[ms]，0=持续动作"
                    }
                },
                "remark": "四腿蹦跳步态"
            },
            303: {
                "name": "慢走",
                "mode": 11,
                "gait_id": 27,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3", "LOCOMOTION=11"],
                "params": {
                    "vel_des": {
                        "type": list,
                        "range": [(-0.60, 0.6), (-0.2, 0.2), (-1.30, 1.30)],
                        "default": [0.0, 0.0, 0.0],
                        "desc": "期望速度[m/s] [x,y,yaw]"
                    },
                    "duration": {
                        "type": int,
                        "range": (0, None),
                        "default": 0,
                        "desc": "时间[ms]，0=持续动作"
                    },
                    "rpy_des": {
                        "type": list,
                        "range": [(-0.25, 0.3), (0, 0), (0, 0)],
                        "default": [0.0, 0.0, 0.0],
                        "desc": "机身俯仰角[rad]"
                    },
                    "pos_des": {
                        "type": list,
                        "range": [(0, 0), (0, 0), (0.1, 0.28)],
                        "default": [0.0, 0.0, 0.25],
                        "desc": "机身高度[m]"
                    },
                    "step_height": {
                        "type": list,
                        "range": [(0, 0.06), (0, 0.06)],
                        "default": [0.06, 0.06],
                        "desc": "抬腿高度[m]"
                    }
                },
                "remark": "慢走步态"
            },
            125: {
                "name": "牵绳遛狗",
                "mode": 11,
                "gait_id": 4,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3", "LOCOMOTION=11"],
                "params": {
                    "vel_des": {
                        "type": list,
                        "range": [(-0.65, 0.4), (-0.4, 0.4), (-0.80, 0.80)],
                        "default": [0.0, 0.0, 0.0],
                        "desc": "期望速度[m/s] [x,y,yaw]"
                    },
                    "duration": {
                        "type": int,
                        "range": (0, None),
                        "default": 0,
                        "desc": "时间[ms]，0=持续动作"
                    }
                },
                "remark": "牵绳遛狗模式步态"
            },
            301: {
                "name": "四足跳跑",
                "mode": 11,
                "gait_id": 7,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3", "LOCOMOTION=11"],
                "params": {
                    "vel_des": {
                        "type": list,
                        "range": [(-0.3, 0.3), (-0.3, 0.3), (-1.5, 1.5)],
                        "default": [0.0, 0.0, 0.0],
                        "desc": "期望速度[m/s] [x,y,yaw]"
                    },
                    "duration": {
                        "type": int,
                        "range": (0, None),
                        "default": 0,
                        "desc": "时间[ms]，0=持续动作"
                    }
                },
                "remark": "四足跳跑步态"
            },
            308: {
                "name": "快走",
                "mode": 11,
                "gait_id": 3,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3", "LOCOMOTION=11"],
                "params": {
                    "vel_des": {
                        "type": list,
                        "range": [(-1.2, 1.2), (-0.4, 0.4), (-2.25, 2.25)],
                        "default": [0.0, 0.0, 0.0],
                        "desc": "期望速度[m/s] [x,y,yaw]"
                    },
                    "duration": {
                        "type": int,
                        "range": (0, None),
                        "default": 0,
                        "desc": "时间[ms]，0=持续动作"
                    }
                },
                "remark": "快走步态"
            },
            305: {
                "name": "小跑",
                "mode": 11,
                "gait_id": 10,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3", "LOCOMOTION=11"],
                "params": {
                    "vel_des": {
                        "type": list,
                        "range": [(-1.2, 1.6), (-0.55, 0.55), (-2.5, 2.5)],
                        "default": [0.0, 0.0, 0.0],
                        "desc": "期望速度[m/s] [x,y,yaw]"
                    },
                    "duration": {
                        "type": int,
                        "range": (0, None),
                        "default": 0,
                        "desc": "时间[ms]，0=持续动作"
                    }
                },
                "remark": "小跑步态"
            },
            304: {
                "name": "变频",
                "mode": 11,
                "gait_id": 26,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3", "LOCOMOTION=11"],
                "params": {
                    "vel_des": {
                        "type": list,
                        "range": [(-1.2, 0.6), (-0.6, 0.6), (-2.5, 2.5)],
                        "default": [0.0, 0.0, 0.0],
                        "desc": "期望速度[m/s] [x,y,yaw]"
                    },
                    "duration": {
                        "type": int,
                        "range": (0, None),
                        "default": 0,
                        "desc": "时间[ms]，0=持续动作"
                    }
                },
                "remark": "变频步态"
            },
            130: {
                "name": "旋转左跳",
                "mode": 16,
                "gait_id": 0,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "QP_STAND=3", "POSE_CTRL=21", "LOCOMOTION=11", "PUREDAMPER=7"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (790, None),
                        "default": 800,
                        "desc": "时间[ms]，必须>790"
                    }
                },
                "remark": "跳跃为单次触发动作，如需连续触发，需要插入其他指令（如站立），时间可以很短（500ms）"
            },
            132: {
                "name": "原地跳远",
                "mode": 16,
                "gait_id": 1,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "QP_STAND=3", "POSE_CTRL=21", "LOCOMOTION=11", "PUREDAMPER=7"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (790, None),
                        "default": 800,
                        "desc": "时间[ms]，必须>790"
                    }
                },
                "remark": "跳跃为单次触发动作，如需连续触发，需要插入其他指令（如站立），时间可以很短（500ms）"
            },
            136: {
                "name": "原地跳起",
                "mode": 16,
                "gait_id": 6,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "QP_STAND=3", "POSE_CTRL=21", "LOCOMOTION=11", "PUREDAMPER=7"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (760, None),
                        "default": 800,
                        "desc": "时间[ms]，必须>760"
                    }
                },
                "remark": "跳跃为单次触发动作，如需连续触发，需要插入其他指令（如站立），时间可以很短（500ms）"
            },
            211: {
                "name": "位控姿态-绝对姿态",
                "mode": 21,
                "gait_id": 5,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3", "LOCOMOTION=11", "MOTION=62", "JUMP3D=16"],
                "params": {
                    "pos_des": {
                        "type": list,
                        "range": [(0.13, 0.32), (0, 0), (0, 0)],
                        "default": [0.25, 0.0, 0.0],
                        "desc": "站立高度[m]"
                    },
                    "duration": {
                        "type": int,
                        "range": (0, None),
                        "default": 0,
                        "desc": "时间[ms]，>0"
                    }
                },
                "remark": "绝对姿态控制"
            },
            212: {
                "name": "位控姿态-相对姿态",
                "mode": 21,
                "gait_id": 0,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3", "LOCOMOTION=11", "MOTION=62", "JUMP3D=16"],
                "params": {
                    "rpy_des": {
                        "type": list,
                        "range": [(-float('inf'), float('inf')), (-float('inf'), float('inf')), (-float('inf'), float('inf'))],
                        "default": [0.0, 0.0, 0.0],
                        "desc": "机身6自由度姿态角[rad]"
                    },
                    "pos_des": {
                        "type": list,
                        "range": [(-float('inf'), float('inf')), (-float('inf'), float('inf')), (-float('inf'), float('inf'))],
                        "default": [0.0, 0.0, 0.0],
                        "desc": "机身6自由度位置[m]"
                    }
                },
                "remark": "相对姿态控制，有超限限位"
            },
            141: {
                "name": "握左手",
                "mode": 62,
                "gait_id": 1,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (8600, None),
                        "default": 8600,
                        "desc": "时间[ms]，必须>8600"
                    }
                },
                "remark": "表演动作：握左手"
            },
            142: {
                "name": "握右手",
                "mode": 62,
                "gait_id": 2,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (8600, None),
                        "default": 8600,
                        "desc": "时间[ms]，必须>8600"
                    }
                },
                "remark": "表演动作：握右手"
            },
            143: {
                "name": "坐下",
                "mode": 62,
                "gait_id": 3,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (1300, None),
                        "default": 1300,
                        "desc": "时间[ms]，必须>1300"
                    }
                },
                "remark": "表演动作：坐下"
            },
            144: {
                "name": "扭屁股",
                "mode": 62,
                "gait_id": 4,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (3900, None),
                        "default": 3900,
                        "desc": "时间[ms]，必须>3900"
                    }
                },
                "remark": "表演动作：扭屁股"
            },
            145: {
                "name": "扭头",
                "mode": 62,
                "gait_id": 5,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (3900, None),
                        "default": 3900,
                        "desc": "时间[ms]，必须>3900"
                    }
                },
                "remark": "表演动作：扭头"
            },
            146: {
                "name": "伸懒腰",
                "mode": 62,
                "gait_id": 6,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (3300, None),
                        "default": 3300,
                        "desc": "时间[ms]，必须>3300"
                    }
                },
                "remark": "表演动作：伸懒腰"
            },
            151: {
                "name": "芭蕾舞",
                "mode": 62,
                "gait_id": 11,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (27000, None),
                        "default": 27000,
                        "desc": "时间[ms]，必须>27000"
                    }
                },
                "remark": "表演动作：芭蕾舞"
            },
            152: {
                "name": "太空步",
                "mode": 62,
                "gait_id": 12,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (9000, None),
                        "default": 9000,
                        "desc": "时间[ms]，必须>9000"
                    }
                },
                "remark": "表演动作：太空步"
            },
            174: {
                "name": "俯卧撑",
                "mode": 62,
                "gait_id": 34,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "POSE_CTRL=21", "QP_STAND=3"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (14000, None),
                        "default": 14000,
                        "desc": "时间[ms]，必须>14000"
                    }
                },
                "remark": "表演动作：俯卧撑"
            },
            123: {
                "name": "作揖",
                "mode": 64,
                "gait_id": 0,
                "allowed_switch": ["RECOVERY_STAND=12", "OFF=0", "PUREDAMPER=7", "JUMP3D=16"],
                "params": {
                    "duration": {
                        "type": int,
                        "range": (8700, None),
                        "default": 8700,
                        "desc": "时间[ms]，必须>8700"
                    }
                },
                "remark": "表演动作：作揖"
            }
        }

    def get_action_info(self, action_id):
        """根据表格原始编号获取动作元数据"""
        if not isinstance(action_id, int):
            raise TypeError(f"动作编号必须是整数，实际传入 {type(action_id).__name__}")
        if action_id not in self.action_meta:
            raise ValueError(f"动作编号 {action_id} 未定义，已定义编号：{sorted(list(self.action_meta.keys()))}")
        return self.action_meta[action_id]

    def validate_params(self, action_id, params):
        """验证动作参数是否符合表格要求"""
        action_info = self.get_action_info(action_id)
        errors = []

        for param_name, param_spec in action_info["params"].items():
            param_value = params.get(param_name, param_spec["default"])

            # 类型检查
            if not isinstance(param_value, param_spec["type"]):
                errors.append(
                    f"【{action_info['name']}({action_id})】参数 {param_name} 类型错误：期望 {param_spec['type'].__name__}，实际 {type(param_value).__name__}")
                continue

            # 范围检查
            if param_spec["range"] is not None:
                if param_spec["type"] == list:
                    if len(param_value) != len(param_spec["range"]):
                        errors.append(
                            f"【{action_info['name']}({action_id})】参数 {param_name} 长度错误：期望 {len(param_spec['range'])} 个元素，实际 {len(param_value)}")
                        continue
                    for i, (val, (min_val, max_val)) in enumerate(zip(param_value, param_spec["range"])):
                        if not (min_val <= val <= max_val):
                            errors.append(
                                f"【{action_info['name']}({action_id})】参数 {param_name}[{i}] 超出范围：期望 [{min_val}, {max_val}]，实际 {val}")
                elif param_spec["type"] == int or param_spec["type"] == float:
                    min_val, max_val = param_spec["range"]
                    if (min_val is not None and param_value < min_val) or (
                            max_val is not None and param_value > max_val):
                        errors.append(
                            f"【{action_info['name']}({action_id})】参数 {param_name} 超出范围：期望 [{min_val}, {max_val}]，实际 {param_value}")
                elif param_spec["type"] == str:
                    if not re.match(param_spec["range"], param_value):
                        errors.append(
                            f"【{action_info['name']}({action_id})】参数 {param_name} 格式错误：期望匹配正则 {param_spec['range']}，实际 {param_value}")

        if errors:
            raise ValueError(f"动作参数验证失败：\n" + "\n".join(errors))
        return params
