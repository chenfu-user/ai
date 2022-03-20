import asyncio
import copy
import socket
from uuid import uuid4
import os
import logging
from raf.Define import max_size, address_head
import pickle
import time


class RafBase:
    def __init__(self, name, loop, address: str = None):
        self.__name = name
        # 日志相关（子类不用管日志的问题了，不过要用self.log发送日志）
        self.log = logging.getLogger(str(self.__class__).split('.')[-1][:-2] + '<' + self.__name + '>')
        logging.basicConfig(level=logging.ERROR,
                            format='%(asctime)s %(name)s (p%(process)d, t%(thread)d): %(message)s')
        if address is None:
            self.__address = address_head + str(uuid4())
        else:
            self.__address = address
        self.__operator = {}
        self.__future = {}
        self.__callback = {}
        # socket相关
        self.__socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        if os.path.exists(self.__address):
            os.unlink(self.__address)
        self.__socket.bind(self.__address)
        self.__create_loop = False
        if loop is not None:
            self.__loop = loop
        else:
            try:
                logging.debug('message create loop %s.', loop)
                loop = asyncio.new_event_loop()
            except RuntimeError as e:
                logging.error('create loop error: %s', e)
            self.__create_loop = True
            self.__loop = loop
        self.__loop.add_reader(self.__socket, self.__on_operator)
        self.log.debug('create %s', str(self))

    @staticmethod
    def _create_address():
        return address_head + str(uuid4())

    def __del__(self):
        self.log.debug('delete %s', str(self))
        if isinstance(self.__socket, socket.socket):
            self.__socket.close()
            if os.path.exists(self.__address):
                os.unlink(self.__address)

    def close(self):
        self.log.debug('close in RafBase %s', str(self))
        if self.__loop is not None:
            self.__loop.remove_reader(self.__socket)
            if self.__loop.is_closed() is False and self.__create_loop is True:
                if self.__loop.is_running():
                    self.__loop.stop()
                self.__loop.call_soon(self.__loop.close)
                # self.__loop.close()

    def __repr__(self):
        return str(self.__class__).split('.')[-1][:-2] + '<' + self.name + '>' + ' address: ' + self.address

    @property
    def name(self):
        return self.__name

    @property
    def address(self):
        return self.__address

    @property
    def loop(self):
        return self.__loop

    @staticmethod
    def check_address(address) -> bool:
        if not isinstance(address, str):
            return False
        return address.startswith(address_head)

    # 异步写
    def send(self, data: dict, address: str) -> bool:
        data['time'] = round(time.time() * 1000)
        self.log.debug('send data %s to %s', data, address)
        # 序列化
        try:
            data = pickle.dumps(data)
        except Exception as e:
            self.log.warning('pickle.dumps data error %s, data: %s', e, data)
            return False
        # 发送
        try:
            ret = self.__socket.sendto(data, address)
        except Exception as e:
            self.log.warning('socket sendto to %s error %s, data: %s', address, e, data)
            return False
        if ret < len(data):
            self.log.warning('socket sendto to %s, need send %s but send %s', address, len(data), ret)
        else:
            return True

    # 同步写
    async def send_and_wait_return(self, data: dict, address: str, timeout: float = 0.1):
        key = str(uuid4())
        data['id'] = key
        ret = self.send(data, address)
        if ret is False:
            return ret
        self.__future[key] = self.__loop.create_future()
        try:
            await asyncio.wait_for(self.__future[key], timeout=timeout, loop=self.__loop)
        except Exception() as e:
            self.__future.pop(key)
            if e is TimeoutError:
                return False
            else:
                self.log.warning('send_and_wait_return data %s to %s error %s', data, address, e)
                return False
        finally:
            return self.__future.pop(key).result()

    # 写并设定返回消息处理器
    def send_set_callback(self, data: dict, address: str, func):
        if not callable(func):
            self.log.warning('send_set_callback param func can not call, %s, %s', data, address)
            return False
        key = str(uuid4())
        data['id'] = key
        ret = self.send(data, address)
        if ret is False:
            return ret
        self.__callback[key] = func
        return True

    @staticmethod
    def __dict_in(d1: dict, d2: dict):
        for k, v in d1.items():
            if k not in d2:
                return False
            if isinstance(v, list):
                if d2[k] != d1[k]:
                    return False
            elif d2[k] != v:
                return False
        return True

    def add_operator(self, op: str, sub: str, func) -> bool:
        if not callable(func):
            self.log.warning('add operator error, op %s func %s can not call', op, func)
            return False
        key = (op, sub)
        self.__operator[key] = func
        return True

    def __on_operator(self):
        # 接收数据
        try:
            data = self.__socket.recv(max_size)
        except Exception as e:
            self.log.error('socket receive error %s', e)
            return False
        # 反序列化
        try:
            data = pickle.loads(data)
        except Exception as e:
            self.log.warning('pickle.loads data error %s, data: %s', e, data)
            return False
        self.log.debug('receive option %s', data)
        if not isinstance(data, dict) or 'op' not in data or 'to_sub' not in data:
            self.log.warning('socket receive param error, param: %s', data)
        # 检测数据
        if not {'op', 'from_sub', 'from', 'to', 'to_sub', 'time'} <= set(data.keys()):
            self.log.error('receive data format error %s.', data)
            return False
        # 处理协程调用
        if 'id' in data and (data['id'] in self.__future or data['id'] in self.__callback):
            if data['id'] in self.__future:
                self.__future[data['id']].set_result(data.get('data', None))
                return True
            if data['id'] in self.__callback:
                try:
                    self.__callback.pop(data['id'])(data.get('data', None))
                except TypeError as e:
                    self.log.warning('function args %s error %s', data, e)
                    return False
                return True
            self.log.warning('unknown answer message %s', data)
            return False
        # 调用用户操作处理器
        key = (data['op'], data['to_sub'])
        if key not in self.__operator:
            self.log.warning('unknown message %s', data)
            return False
        else:
            func = self.__operator[key]
            try:
                ret = func(data)
            except TypeError as e:
                self.log.error('call user operator function %s args %s error %s.',
                               func, data, e)
                return False
            return ret
