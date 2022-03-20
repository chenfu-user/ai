from raf.SubBase import InBase, InGhostBase, OutBase, OutGhostBase
from raf.RafBase import RafBase


def change_args(*args, **kwargs):
    # 一个字典参数
    if len(args) == 1 and len(kwargs) == 0 and isinstance(args[0], dict):
        data = args[0]
    # 使用关键字参数
    elif len(args) == 0 and len(kwargs) > 0:
        data = kwargs
    # 没有参数
    elif len(args) == 0 and len(kwargs) == 0:
        data = {}
    # 其他情况
    else:
        return False
    return data


class Signal(OutBase):
    def __init__(self, name: str, node):
        super().__init__('signal', name, node)

    def __call__(self, *args, **kwargs):
        return self.send(*args, **kwargs)

    def send(self, *args, **kwargs):
        param = change_args(*args, **kwargs)  # 待发送数据内容
        data = {
            'op': 'message',
            'data': param
        }
        return super().write(data)


class Function(InBase):
    def __init__(self, name, node):
        super().__init__('function', name, node)
        self.__function = None
        super().set_reader(self.__on_function)

    def __on_function(self, data):
        if 'data' in data and isinstance(data['data'], dict):
            return self(data['data'])

    def __call__(self, *args, **kwargs):
        data = change_args(*args, **kwargs)
        if callable(self.__function):
            try:
                ret = self.__function(**data)
            except ValueError as e:
                self.log.warning('call function %s error %s, param %s', self.name, e, data)
                ret = False
            return ret
        return False

    def set_function(self, func=None):
        if func is None:
            def _set_reader(_func):
                if callable(_func):
                    self.log.debug('set user function %s in %s', _func, self)
                    self.__function = _func
                else:
                    self.log.warning('set user function %s in %s param error', _func, self)
                return _func
            return _set_reader
        if callable(func):
            self.__function = func
            self.log.debug('set user function %s in %s', func, self)
            return True
        else:
            self.log.warning('set user function %s in %s param error', func, self)
            return False


class SignalGhost(OutGhostBase):
    def __init__(self, name, address, node: RafBase):
        super().__init__('signal', name, address, node)


class FunctionGhost(InGhostBase):
    def __init__(self, name, address, node: RafBase):
        super().__init__('function', name, address, node)

    def __call__(self, *args, **kwargs):
        param = change_args(*args, **kwargs)
        data = {
            'op': 'message',
            'data': param,
            'from': self.ghost_address
        }
        return super()._write(data, self.address, self.name)

    async def call_sync(self, param, timeout: float = 0.1):
        data = {
            'op': 'message',
            'data': param,
            'from': self.ghost_address
        }
        return await super()._write_and_wait_return(data, self.address, self.name, timeout=timeout)

    def call_set_callback(self, param, func):
        data = {
            'op': 'message',
            'data': param,
            'from': self.ghost_address
        }
        return super()._write_and_wait_return(data, self.address, self.name, func)
