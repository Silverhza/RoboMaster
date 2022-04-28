# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from roborts_msgs/ShootCmdRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ShootCmdRequest(genpy.Message):
  _md5sum = "8d21b018438e79ef291ba097490af2e5"
  _type = "roborts_msgs/ShootCmdRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint8 STOP = 0
uint8 ONCE = 1
uint8 CONTINUOUS = 2
uint8 mode
uint8 number
uint16 freq
"""
  # Pseudo-constants
  STOP = 0
  ONCE = 1
  CONTINUOUS = 2

  __slots__ = ['mode','number','freq']
  _slot_types = ['uint8','uint8','uint16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       mode,number,freq

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ShootCmdRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.mode is None:
        self.mode = 0
      if self.number is None:
        self.number = 0
      if self.freq is None:
        self.freq = 0
    else:
      self.mode = 0
      self.number = 0
      self.freq = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_2BH().pack(_x.mode, _x.number, _x.freq))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 4
      (_x.mode, _x.number, _x.freq,) = _get_struct_2BH().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2BH().pack(_x.mode, _x.number, _x.freq))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 4
      (_x.mode, _x.number, _x.freq,) = _get_struct_2BH().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2BH = None
def _get_struct_2BH():
    global _struct_2BH
    if _struct_2BH is None:
        _struct_2BH = struct.Struct("<2BH")
    return _struct_2BH
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from roborts_msgs/ShootCmdResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ShootCmdResponse(genpy.Message):
  _md5sum = "dd4152e077925db952c78baadb1e48b7"
  _type = "roborts_msgs/ShootCmdResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool received

"""
  __slots__ = ['received']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       received

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ShootCmdResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.received is None:
        self.received = False
    else:
      self.received = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.received
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.received,) = _get_struct_B().unpack(str[start:end])
      self.received = bool(self.received)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.received
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.received,) = _get_struct_B().unpack(str[start:end])
      self.received = bool(self.received)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
class ShootCmd(object):
  _type          = 'roborts_msgs/ShootCmd'
  _md5sum = '8a7441c08a86e123da484ba84c6284f7'
  _request_class  = ShootCmdRequest
  _response_class = ShootCmdResponse
