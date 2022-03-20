# coding=utf-8
from raf.RafBase import RafBase
from raf.Pad import InPad, OutPad, InPadGhost, OutPadGhost
from raf.Message import Signal, Function, SignalGhost, FunctionGhost
from raf.Define import paths
import asyncio
import logging
import os
import json
import random
import multiprocessing
import threading
import importlib
import raf
import atexit
import time


# 用于实际运行环境中运行
# 配置文件格式：
# {
#     'template': 'node template',
#     'des': '组件描述（给人看的）',
#     'type': 'node',
#     'environment': 'main / thread / process',
#     'pad': [
#         {
#             'name': 'xxx',
#             'type': 'out',
#             'max_size': 123,
#             'format': {}
#         },
#         {
#             'name': 'xxx',
#             'type': 'in',
#             'format': {}
#         },
#         {
#             'name': 'xxx',
#             'type': 'in',
#             'format': {}
#         }
#     ],
#     'signal': {
#         'name 1': {
#             'arg name': 'type'
#         },
#         'name 2': {
#             'arg name': 'type'
#         }
#     },
#     'function': {
#         'name 1': {
#             'arg name': 'type'
#         },
#         'name 2': {
#             'arg name': 'type'
#         }
#     }
# }
class NodeBase(RafBase):
    def __init__(self, name: str, cfg: dict, loop=None, address: str = None):
        # if loop is None:
        #     try:
        #         loop = asyncio.get_running_loop()
        #     except RuntimeError as e:
        #         logging.warning('get running loop warning: %s', e)
        #         loop = asyncio.get_event_loop()
        #         logging.debug('message create loop %s.', loop.__dict__)
        super().__init__(name, loop, address)
        self._pads = {}    # dict
        self._signals = {}
        self._functions = {}
        if cfg is not None:
            # 创建pad
            if 'pad' in cfg:
                for i in cfg.get('pad', []):
                    self._add_pad(i)
            # 创建signal
            if 'signal' in cfg:
                for k, v in cfg.get('signal', {}).items():
                    self._add_signal(k)
            # 创建function
            if 'function' in cfg:
                for k, v in cfg.get('function', {}).items():
                    self._add_function(k)
            # 增加默认的setting功能，用于对node进行设置，其会在系统初始化后由系统自动调用
            if 'setting' not in self._functions:
                self._add_function('setting')
            # close
            self._add_function('close')

    def _add_pad(self, cfg: dict):
        pass

    def _add_signal(self, name: str):
        pass

    def _add_function(self, name: str):
        pass

    @property
    def pads(self):
        return self._pads

    @property
    def signals(self):
        return self._signals

    @property
    def functions(self):
        return self._functions


class Node(NodeBase):
    def __init__(self, name, cfg: dict = None, loop=None, address: str = None):
        super().__init__(name, cfg, loop, address)
        self.functions['close'].set_function(self.close)
        self.__nodes = dict()

    def close(self):
        self.log.debug('close in Node %s', str(self))
        for i in self.__nodes.values():
            i.close()
        super().close()

    @property
    def nodes(self):
        return self.__nodes

    # 创建pad
    def _add_pad(self, cfg: dict):
        if 'type' not in cfg or 'name' not in cfg or cfg.get('type') not in ('in', 'out'):
            self.log.warning('pad config error %s', cfg)
            return
        pad = {
            'in': InPad,
            'out': OutPad
        }
        name = cfg.get('name')
        self._pads[name] = pad[cfg.get('type')](cfg, self)
        setattr(self, name, self._pads[name])

    # 创建signal
    def _add_signal(self, name: str):
        self._signals[name] = Signal(name, self)
        setattr(self, name, self._signals[name])

    # 创建function
    def _add_function(self, name: str):
        self._functions[name] = Function(name, self)
        if hasattr(self, name) is False:
            setattr(self, name, self._functions[name])

    # 创建node
    # 文件组织：
    #   1、node存在于base、plugin、user三个目录中，以node template为文件名
    #   2、每个node至少包含config.json和main.py两个文件，config包含node配置，main为用户程序入口
    #   3、搜索顺序为 base -> plugin -> user
    # 创建node后可直接调用setting功能，进行对node的初始设置
    def create_node(self, template: str, *args, **kwargs):
        # 检测文件和配置
        # 检测template是否存在
        path = __file__[0: -(len(__file__.split('/')[-1]) + len(__file__.split('/')[-2]) + 1)]
        for i in paths:
            if os.path.exists(path + i + template):
                path += i + template + '/'
        if template not in path:
            self.log.warning('can not find node file %s', template)
            return False
        # 检测 config.json & main.py 文件是否存在
        if not os.path.exists(path + 'config.json') or not os.path.exists(path + 'main.py'):
            self.log.warning('can not find config or main file in node %s', template)
            return False
        # 读取配置文件
        with open(path + 'config.json') as file:
            cfg = file.read()
        # 解析配置信息
        try:
            cfg = json.loads(cfg)
        except Exception as e:
            self.log.warning('loads config.json %s error %s', cfg, e)
            return False
        # 检测配置要素
        if 'template' not in cfg:
            self.log.warning('node config error, need template, %s', cfg)
            return False
        # 创建NodeGhost实例，实例由用户管理
        name = cfg.get('template') + str(random.randint(0, 100))
        while name in self.__nodes:
            name = cfg.get('template') + str(random.randint(0, 100))
        cfg['path'] = path
        self.__nodes[name] = NodeGhost(name, cfg, self.loop)
        self.__nodes[name].setting(*args, **kwargs)
        return self.__nodes[name]


# 用于在上级节点中管理
class NodeGhost(NodeBase):
    def __init__(self, name: str, cfg: dict, loop=None, address: str = None):
        self.__environment = cfg.get('environment', 'process')
        self.__node_address = super()._create_address()
        if self.__environment == 'main':
            self.__node = create_node(name, cfg, self.__node_address, loop)
        elif self.__environment == 'thread':
            self.__node = threading.Thread(target=create_node, args=(name, cfg, self.__node_address))
            self.__node.start()
        elif self.__environment == 'process':
            self.__node = multiprocessing.Process(target=create_node, args=(name, cfg, self.__node_address))
            self.__node.start()
        super().__init__(name, cfg, loop, address)

    def __del__(self):
        if hasattr(self.__node, 'kill'):
            self.__node.kill()
        # elif isinstance(self.__node, Node):
        #     self.__node.close()
        del self.__node
        self.__node = None
        super().__del__()

    # def __repr__(self):
    #     ret = super(NodeGhost, self).__repr__()
    #     ret += ', ' + self.address
    #     return ret

    def close(self):
        self.log.debug('close in NodeGhost %s', str(self))
        self.functions['close']()
        # time.sleep(0.02)
        super().close()

    # 创建pad
    def _add_pad(self, cfg: dict):
        if 'type' not in cfg or 'name' not in cfg or cfg.get('type') not in ('in', 'out'):
            self.log.warning('pad config error %s', cfg)
            return
        pad = {
            'in': InPadGhost,
            'out': OutPadGhost
        }
        name = cfg.get('name')
        self._pads[name] = pad[cfg.get('type')](name, self.__node_address, self)
        setattr(self, name, self._pads[name])

    # 创建signal
    def _add_signal(self, name: str):
        self._signals[name] = SignalGhost(name, self.__node_address, self)
        setattr(self, name, self._signals[name])

    # 创建function
    def _add_function(self, name: str):
        self._functions[name] = FunctionGhost(name, self.__node_address, self)
        if hasattr(self, name) is False:
            setattr(self, name, self._functions[name])


def create_node(_name: str, _cfg: dict, address: str, loop=None):
    global node
    node = Node(_name, _cfg, loop, address)
    raf.node = node
    p = _cfg['path'].split('/')
    importlib.import_module('.main', package=p[-3] + '.' + p[-2])
    atexit.register(node.close)
    if node.loop.is_running() is False:
        node.loop.run_forever()
    return node


node = None


class App(Node):
    __app = []

    def __init__(self):
        super().__init__('app', {})
        self.__app.append(self)
        atexit.register(self.__class__.__close)  # 将func 注册为终止时执行的函数

    @classmethod
    def __close(cls):
        logging.debug('__close in App')
        for i in cls.__app:
            i.close()



