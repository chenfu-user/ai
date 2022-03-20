import logging

from raf.RafBase import RafBase
from raf.Define import max_size, address_head, type_name


# key = (address, sub_name)
class Base:
    def __init__(self, sub_type, name, node: RafBase):
        if sub_type not in type_name:
            node.log.error('type name error,type %s', sub_type)
        self.__type = sub_type
        self.__name = name
        self.__node = node
        self.__log = self.__node.log

    def __repr__(self):
        ret = str(self.__class__).split('.')[-1][:-2] + '<' + self.name + '>' + ': ' + self.address
        return ret

    @property
    def name(self):
        return self.__name

    @property
    def type(self):
        return self.__type

    @property
    def address(self):
        return self.__node.address

    @property
    def key(self):
        ret = (self.address, self.name)
        return ret

    @property
    def log(self):
        return self.__log

    def get_key(self, data):
        if isinstance(data, Base):
            return data.key
        elif isinstance(data, dict) \
                and 'data' in data \
                and isinstance(data['data'], dict) \
                and 'address' in data['data'] \
                and 'sub_name' in data['data']:
            ret = (data['data']['address'], data['data']['sub_name'])
            if self.check_key(ret):
                return ret
            return None
        return None

    def check_key(self, data):
        if not isinstance(data, tuple) \
                or len(data) != 2 \
                or not self.__node.check_address(data[0]) \
                or not isinstance(data[1], str) \
                or len(data[1]) <= 0:
            return False
        return True

    def __send_data_change(self, data, address: str, sub: str = None):
        data['to'] = address
        data['to_sub'] = sub
        data['from'] = self.address
        data['from_sub'] = self.name
        return data, address

    def _write(self, data, address: str = None, sub: str = None):
        # import time
        # time.sleep(0.01)
        if address is None:
            return self.__node.send(data, data['to'])
        else:
            return self.__node.send(*self.__send_data_change(data, address, sub))

    async def _write_and_wait_return(self, data, address: str, sub: str = None, timeout: float = 0.1):
        data, address = self.__send_data_change(data, address, sub)
        return await self.__node.send_and_wait_return(data, address, timeout)

    def _write_set_callback(self, data: dict, address: str, sub: str, func):
        data, address = self.__send_data_change(data, address, sub)
        return self.__node.send_set_callback(data, address, func)

    def _add_operator(self, op, sub, func):
        return self.__node.add_operator(op, sub, func)


class SubBase(Base):
    def __init__(self, sub_type, name, node: RafBase):
        super().__init__(sub_type, name, node)
        super()._add_operator('message', self.name, self.__on_message)
        super()._add_operator('answer', self.name, self.__on_answer)
        super()._add_operator('add_proxy', self.name, self.add_proxy)
        super()._add_operator('del_proxy', self.name, self.del_proxy)
        self.__proxy = None
        self.__message_processor = None

    def __on_message(self, data: dict):
        if self.__proxy is not None:
            data['to_sub'] = self.__proxy[1]
            data['to'] = self.__proxy[0]
            return self._write(data, data['to'])
        if callable(self.__message_processor):
            try:
                return self.__message_processor(data)
            except TypeError as e:
                self.log.warning('function %s args %s error %s', self.name, data, e)
                return False
        return False

    def _set_message_processor(self, func):
        if not callable(func):
            self.log.warning('set_message_processor param error in %s, param %s', self, func)
            return False
        self.__message_processor = func
        return True

    def __on_answer(self, data: dict):
        if self.__proxy is not None:
            data['to_sub'] = self.__proxy[1]
            data['to'] = self.__proxy[0]
            return self._write(data, data['to'])
        else:
            self.log.warning('unknown answer %s in %s', data, self)
            return False

    def add_proxy(self, key):
        key = self.get_key(key)
        if key is None:
            return False
        self.log.debug('add proxy %s in %s', key, self)
        self.__proxy = key
        return True

    def del_proxy(self, key=None):
        self.__proxy = None
        return True


class GhostBase(Base):
    def __init__(self, sub_type, name, address, node: RafBase):
        super().__init__(sub_type, name, node)
        self.__address = address

    @property
    def ghost_address(self):
        return super().address

    @property
    def address(self):
        return self.__address

    def __send_proxy(self, op, data):
        key = self.get_key(data)
        if op == 'add_proxy' and self.check_key(key) is False:
            self.log.warning('%s param error in %s, param %s', op, self, key)
            return False
        send = {
            'op': op,
            'data': None
        }
        if op == 'add_proxy':
            send['data'] = {
                'address': key[0],
                'sub_name': key[1]
            }
        return self._send2real(send)

    def add_proxy(self, key):
        return self.__send_proxy('add_proxy', key)

    def del_proxy(self, key=None):
        return self.__send_proxy('del_proxy', key)

    def _send2real(self, data):
        data['from'] = self.ghost_address
        data['from_sub'] = self.name
        data['to'] = self.address
        data['to_sub'] = self.name
        self.log.debug('send to real %s', data)
        # return super()._write(data, self.address)
        return super()._write(data)


def proxy(sup, sub):
    if not hasattr(sup, 'log') \
            or not hasattr(sup, 'type') \
            or not hasattr(sub, 'type'):
        if hasattr(sup, 'type'):
            log = sup.log
        else:
            log = logging
        log.warning('proxy param error, param %s, %s', sup, sub)
        return False
    if not isinstance(sup.type, str) \
            or not isinstance(sub.type, str) \
            or sup.type != sup.type:
        sup.log.warning('proxy param error, param %s, %s', sup, sub)
        return False
    if sup.type in {'in', 'function'}:
        if not hasattr(sup, 'add_proxy') \
                or not callable(sup.add_proxy):
            sup.log.warning('proxy param error, param %s, %s', sup, sub)
            return False
        return sup.add_proxy(sub)
    elif sup.type in {'out', 'signal'}:
        if not hasattr(sub, 'add_proxy') \
                or not callable(sub.add_proxy) \
                or not callable(sup.add_subscribe):
            sup.log.warning('proxy param error, param %s, %s', sup, sub)
            return False
        if sup.add_subscribe(sub) is False:
            return False
        return sub.add_proxy(sup)
    else:
        sup.log.warning('proxy param error, param %s, %s', sup, sub)
        return False


def remove_proxy(sup, sub):
    if not hasattr(sup, 'log') \
            or not hasattr(sup, 'type') \
            or not hasattr(sub, 'type'):
        if hasattr(sup, 'log'):
            log = sup.log
        else:
            log = logging
        log.warning('remove proxy param error, param %s, %s', sup, sub)
        return False
    if not isinstance(sup.type, str) \
            or not isinstance(sub.type, str) \
            or sup.type != sup.type:
        sup.log.warning('remove proxy param error, param %s, %s', sup, sub)
        return False
    if sup.type in {'in', 'function'}:
        if not hasattr(sup, 'del_proxy') \
                or not callable(sup.del_proxy):
            sup.log.warning('remove proxy param error, param %s, %s', sup, sub)
            return False
        return sup.del_proxy(sub)
    elif sup.type in {'out', 'signal'}:
        if not hasattr(sub, 'del_proxy') \
                or not callable(sub.del_proxy) \
                or not callable(sup.del_subscribe):
            sup.log.warning('remove proxy param error, param %s, %s', sup, sub)
            return False
        sup.del_subscribe(sub)
        return sub.del_proxy(sup)
    else:
        sup.log.warning('remove proxy param error, param %s, %s', sup, sub)
        return False


class InBase(SubBase):
    def __init__(self, sub_type, name, node: RafBase):
        super().__init__(sub_type, name, node)
        self.__reader = None
        super()._set_message_processor(self.__on_reader)

    def set_reader(self, func=None):
        if func is None:
            def _set_reader(_func):
                if callable(_func):
                    self.log.debug('set reader %s in %s', _func, self)
                    self.__reader = _func
                else:
                    self.log.warning('set reader %s in %s param error', _func, self)
                return _func

            return _set_reader
        if callable(func):
            self.__reader = func
            self.log.debug('set reader %s in %s', func, self)
            return True
        else:
            self.log.warning('set reader %s in %s param error', func, self)
            return False

    def __on_reader(self, data: dict):
        ret = False
        if callable(self.__reader):
            try:
                ret = self.__reader(data)
            except TypeError as e:
                self.log.error('call user operator function %s args %s error %s.',
                               self.__reader, data, e)
            if isinstance(ret, tuple):
                ret = list(ret)
        if 'id' in data:
            data['op'] = 'answer'
            data['data'] = ret
            data.pop('time')
            return self._write(data, data['from'])


class OutBase(SubBase):
    def __init__(self, sub_type, name, node: RafBase):
        super().__init__(sub_type, name, node)
        self.__subscribe = set()
        super()._add_operator('add_subscribe', self.name, self.add_subscribe)
        super()._add_operator('del_subscribe', self.name, self.del_subscribe)

    def add_subscribe(self, data):
        key = self.get_key(data)
        if key is not None:
            self.log.debug('add subscribe %s in %s', key, self)
            self.__subscribe.add(key)
            return True
        else:
            self.log.warning('add subscribe param %s error in %s', data, self)
            return False

    def del_subscribe(self, data):
        key = self.get_key(data)
        if key is None:
            self.log.debug('clear subscribe in %s', self)
            self.__subscribe.clear()
            return True
        if key in self.__subscribe:
            self.log.debug('del subscribe %s in %s', key, self)
            self.__subscribe.remove(key)
            return True
        else:
            self.log.warning('del subscribe param %s error in %s', data, self)
            return False

    def write(self, data):
        send = data
        for address, sub in self.__subscribe:
            ret = super()._write(send, address, sub)
            if ret is False:
                self.log.warning('send message %s to %s, %s in %s', data, address, sub, self)
        return True


class InGhostBase(GhostBase):
    def __init__(self, sub_type, name, address, node: RafBase):
        super().__init__(sub_type, name, address, node)


class OutGhostBase(GhostBase):
    def __init__(self, sub_type, name, address, node: RafBase):
        super().__init__(sub_type, name, address, node)

    def __send_subscribe(self, op, data):
        key = self.get_key(data)
        if op == 'add_subscribe' and self.check_key(key) is False:
            self.log.warning('%s param error in %s, param %s', op, self, key)
            return False
        send = {
            'op': op,
            'data': None
        }
        if self.check_key(key):
            send['data'] = {
                'address': key[0],
                'sub_name': key[1]
            }
        return self._send2real(send)

    def add_subscribe(self, data):
        return self.__send_subscribe('add_subscribe', data)

    def del_subscribe(self, data):
        return self.__send_subscribe('del_subscribe', data)


def link(src, sink):
    if not hasattr(src, 'type') \
            or not hasattr(sink, 'type') \
            or not isinstance(src.type, str) \
            or not isinstance(sink.type, str) \
            or src.type not in {'out', 'signal'} \
            or sink.type not in {'in', 'function'} \
            or not hasattr(src, 'add_subscribe') \
            or not callable(src.add_subscribe):
        if hasattr(src, 'log'):
            log = src.log
        else:
            log = logging
        log.warning('link param error, src %s, sink %s', src, sink)
        return False
    return src.add_subscribe(sink)


def unlink(src, sink=None):
    if not hasattr(src, 'type') \
            or not isinstance(src.type, str) \
            or src.type not in {'out', 'signal'} \
            or not hasattr(src, 'del_subscribe') \
            or not callable(src.del_subscribe):
        if hasattr(src, 'log'):
            log = src.log
        else:
            log = logging
        log.warning('unlink param error, src %s, sink %s', src, sink)
        return False
    return src.del_subscribe(sink)
