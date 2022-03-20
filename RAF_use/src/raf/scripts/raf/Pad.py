from raf.SubBase import InBase, InGhostBase, OutBase, OutGhostBase
from raf.RafBase import RafBase
from multiprocessing.shared_memory import SharedMemory
from raf.Define import max_size
import logging
import pickle


class BufferNum(int):
    def __new__(cls, size, value=0, *args, **kwargs):
        obj = super(cls, cls).__new__(cls, value)
        obj.__size = size
        return obj

    def __repr__(self):
        return '<BufferNum: ' + str(int(self)) + ' in ' + str(self.__size) + '>'

    def __change(self, data: int) -> int:
        while data < 0:
            data += self.__size
        while data >= self.__size:
            data -= self.__size
        return data

    def __add__(self, other):
        ret = super(BufferNum, self).__add__(other)
        return self.__class__(self.__size, self.__change(ret))

    def __sub__(self, other):
        ret = super(BufferNum, self).__sub__(other)
        return self.__class__(self.__size, self.__change(ret))

    def __mul__(self, other):
        ret = super(BufferNum, self).__mul__(other)
        return self.__class__(self.__size, self.__change(ret))

    def __truediv__(self, other):
        ret = super(BufferNum, self).__truediv__(other)
        return self.__class__(self.__size, self.__change(ret))

    def size(self):
        return self.__size


class Memory:
    def __init__(self, name: str = None, size: int = 0,
                 create: bool = False, buffer_num: int = 1):
        self.__create = create
        self.__name = name
        self.__size = size
        self.__buffer_num = buffer_num
        self.__index = BufferNum(buffer_num)
        if name is None and create is True and size > 0:
            self.__size = size + 1024 - size % 1024
            # 创建新共享内存
            self.__memory = SharedMemory(create=create, size=self.__size * self.__buffer_num)
            self.__name = self.__memory.name
        elif name is not None and create is False:
            # 连接已创建好的共享内存
            self.__memory = SharedMemory(name=name, create=create)
            self.__size = int(self.__memory.size / self.__buffer_num)
        else:
            logging.critical('create shared memory error! name: %s, need create but no size',
                             self.__name)
            raise ValueError('Memory param error')

    def __del__(self):
        self.__memory.close()
        if isinstance(self.__memory, SharedMemory) and self.__create is True:
            logging.debug('unlink memory %s', self.__memory)
            try:
                self.__memory.unlink()
            except Exception as e:
                logging.warning('unlink memory %s error %s', self, e)
        self.__memory = None

    def __repr__(self):
        return 'Memory(' + str(self.__memory) + ', ' + str(self.__index) + ')'

    @property
    def name(self) -> str:
        return self.__name

    @property
    def size(self) -> int:
        return self.__size

    def write(self, data: bytes) -> any:
        if len(data) > self.__size:
            logging.error('send data over size!')
            return False
        self.__index += 1
        self.__memory.buf[self.__size*int(self.__index):self.__size*int(self.__index)+len(data)] = data
        return int(self.__index)

    def set_index(self, index: int):
        self.__index = BufferNum(self.__index.size(), index)

    # 当需要获取过往数据时，index可传-1，-2，-3，需要注意
    def read(self, size=None, bias=0):
        if size is not None and size > self.__size:
            logging.error('read data over size!')
            return False

        index = self.__index   # 不清楚self.__index值，在Memory重新构建时
        index += bias
        if size is None:
            size = self.__size
        return self.__memory.buf[self.__size * int(index): self.__size * int(index) + size]


class InPad(InBase):

    def __init__(self, cfg: dict, node):
        super().__init__('in', cfg.get('name'), node)
        self.__memory = None
        self.__format = cfg.get('format')
        self.__buffer = {}
        # 数据解析处理器
        self.__read = {
            'socket': self.__socket_read,
            'memory': self.__memory_read,
            'buffer': self.__buffer_read,
        }
        self.__pad_reader = None
        super().set_reader(self.__on_pad_reader)

    def set_reader(self, func=None):
        if func is None:
            def _set_reader(_func):
                if callable(_func):
                    self.log.debug('set pad reader %s in %s', _func, self)
                    self.__pad_reader = _func
                else:
                    self.log.warning('set pad reader %s in %s param error', _func, self)
                return _func

            return _set_reader
        if callable(func):
            self.__pad_reader = func
            self.log.debug('set pad reader %s in %s', func, self)
            return True
        else:
            self.log.warning('set pad reader %s in %s param error', func, self)
            return False

    def __on_pad_reader(self, data: dict):
        if data.get('mode') not in self.__read:
            self.log.warning('receive unknown message %s', data)
            return
        # 调用专用处理函数
        ret = self.__read[data['mode']](data)
        if ret is False:
            return
        data['data'] = ret
        self.__buffer = data
        self.log.debug('receive data %s', data)
        if callable(self.__pad_reader):
            try:
                self.__pad_reader(ret)
            except TypeError as e:
                self.log.error('call reader function %s args %s error %s.',
                               self.__pad_reader, ret, e)

    def __loads(self, data):
        if data is False:
            return False
        try:
            data = pickle.loads(data)
        except Exception as e:
            self.log.warning('pickle.loads data error %s, data: %s', e, data)
            return False
        return data

    def __socket_read(self, data):
        if 'data' not in data:
            self.log.error('receive message format error, data: %s', data)
            return False
        return self.__loads(data['data'])

    def __memory_read(self, data):
        if 'memory' not in data:
            self.log.error('receive message format error, data: %s', data)
            return False
        if isinstance(self.__memory, Memory) is False:
            try:
                self.__memory = Memory(name=data['memory'])   # 不需要buffer_num=data['buffer_num']
            except Exception() as e:
                self.log.error('create memory error %s', e)
                return False
        elif self.__memory.name != data['memory']:
            del self.__memory
            try:
                self.__memory = Memory(name=data['memory'])
            except Exception() as e:
                self.log.error('create memory error %s', e)
                return False
        ret = self.__memory.read(size=data.get('size', None))
        return self.__loads(ret)

    def __buffer_read(self, data: dict):
        if not {'memory', 'buffer_index', 'buffer_num'} <= set(data.keys()):
            self.log.error('receive message format error, data: %s', data)
            return False
        if isinstance(self.__memory, Memory) is False:
            try:
                self.__memory = Memory(name=data['memory'], buffer_num=data['buffer_num'])
            except Exception() as e:
                self.log.error('create memory error %s', e)
                return False
        elif self.__memory.name != data['memory']:
            del self.__memory
            try:
                self.__memory = Memory(name=data['memory'], buffer_num=data['buffer_num'])
            except Exception() as e:
                self.log.error('create memory error %s', e)
                return False

        if 'buffer_index' in data:
            self.__memory.set_index(data['buffer_index'])

        if 'data' in data:
            ret = data['data']
        else:
            ret = self.__memory.read(data['size'])
        return self.__loads(ret)

    # 直接读取最新消息
    # index     0: 当前消息     -1：比当前早一次的消息    -2：...
    def read(self, bias: int = 0):
        if len(self.__buffer) <= 0:
            self.log.warning('no message buffer can read')
            return False
        if bias > 0:
            self.log.warning('bias should < 0')
            return False
        if bias < 0 and self.__buffer.get('mode', '') != 'buffer':
            self.log.warning('only buffer message can use index')
            return False
        # 处理mode为socket和memory的message
        if self.__buffer.get('mode', '') != 'buffer'\
                or (self.__buffer.get('mode', '') == 'buffer' and bias == 0):
            return self.__buffer.get('data', False)
        # 处理mode为buffer的message
        if isinstance(self.__memory, Memory) is False:
            self.log.error('shared memory error')
            return False
        ret = self.__memory.read(bias=bias)
        return self.__loads(ret)


class OutPad(OutBase):

    def __init__(self, cfg: dict, node):
        super().__init__('out', cfg.get('name'), node)
        self.__memory = None
        self.__format = cfg.get('format')
        self.__write = {
            'socket': self.__socket_write,
            'memory': self.__memory_write,
            'buffer': self.__buffer_write
        }
        # 共享内存相关
        # 使用共享内存 情况1：有buffer
        if 'buffer_num' in cfg and cfg['buffer_num'] > 1:
            self.__mode = 'buffer'
            self.__buffer_num = cfg['buffer_num']
        # 使用共享内存 情况2：无buffer
        elif 'max_size' in cfg and cfg['max_size'] > max_size:
            self.__mode = 'memory'
            self.__memory = Memory(
                create=True,
                size=int(cfg['max_size'])
            )
            self.log.debug('create memory %s', self.__memory)
        # 使用socket
        else:
            self.__mode = 'socket'

    def __socket_write(self, data) -> bool:
        size = len(data)
        # 判断是不是需要用共享内存
        if size > max_size:
            return self.__memory_write(data)
        # 封包
        data = {
            'op': 'message',
            'mode': 'socket',
            'size': size,
            'data': data
        }
        # 发送
        return super().write(data)

    def __memory_write(self, data) -> bool:
        size = len(data)
        # 检测共享内存
        if isinstance(self.__memory, Memory) is False or self.__memory.size < size:
            if isinstance(self.__memory, Memory):
                self.log.debug('delete memory %s', self.__memory)
                del self.__memory
            # 创建共享内存
            self.__memory = Memory(create=True, size=size)
            self.log.debug('create memory %s', self.__memory)
        # 发送数据
        if self.__memory.write(data) is False:
            return False
        # 通知封包
        notify = {
            'op': 'message',
            'mode': 'memory',
            'memory': self.__memory.name,
            'size': size,
        }
        # 发送通知
        return super().write(notify)

    def __buffer_write(self, data) -> bool:
        size = len(data)
        # 检测共享内存
        if not isinstance(self.__memory, Memory) \
                or self.__memory.size < size:
            if self.__memory is not None:
                self.log.debug('delete memory %s', self.__memory)
                del self.__memory
            # 创建共享内存
            self.__memory = Memory(create=True, size=size, buffer_num=self.__buffer_num)
            self.log.debug('create memory %s', self.__memory)
        # 发送数据
        index = self.__memory.write(data)
        if index is False:
            return False
        # 通知封包
        notify = {
            'op': 'message',
            'mode': 'buffer',
            'memory': self.__memory.name,
            'size': size,
            'buffer_index': index,
            'buffer_num': self.__buffer_num,
        }
        if size <= max_size:
            notify['data'] = data
        return super().write(notify)

    def write(self, data: dict) -> bool:
        if self.__mode not in self.__write:
            self.log.error('mode error %s', self.__mode)
            return False
        self.log.debug('send data %s', data)
        try:
            data = pickle.dumps(data)
        except Exception as e:
            logging.warning('pickle.dumps data error %s, data: %s', e, data)
            return False
        if callable(self.__write[self.__mode]):
            return self.__write[self.__mode](data)
        return False


class InPadGhost(InGhostBase):
    def __init__(self, name, address, node: RafBase):
        super().__init__('in', name, address, node)


class OutPadGhost(OutGhostBase):
    def __init__(self, name, address, node: RafBase):
        super().__init__('out', name, address, node)
