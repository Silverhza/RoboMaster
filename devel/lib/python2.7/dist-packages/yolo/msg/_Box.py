# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from yolo/Box.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Box(genpy.Message):
  _md5sum = "a88885b4a916fee405bb38cc0f4a8fce"
  _type = "yolo/Box"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float32 xmin # xmin
float32 ymin  # ymin
float32 xmax #xmax
float32 ymax #ymax

float32 confidence 
string obj
float32 distance
"""
  __slots__ = ['xmin','ymin','xmax','ymax','confidence','obj','distance']
  _slot_types = ['float32','float32','float32','float32','float32','string','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       xmin,ymin,xmax,ymax,confidence,obj,distance

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Box, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.xmin is None:
        self.xmin = 0.
      if self.ymin is None:
        self.ymin = 0.
      if self.xmax is None:
        self.xmax = 0.
      if self.ymax is None:
        self.ymax = 0.
      if self.confidence is None:
        self.confidence = 0.
      if self.obj is None:
        self.obj = ''
      if self.distance is None:
        self.distance = 0.
    else:
      self.xmin = 0.
      self.ymin = 0.
      self.xmax = 0.
      self.ymax = 0.
      self.confidence = 0.
      self.obj = ''
      self.distance = 0.

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
      buff.write(_get_struct_5f().pack(_x.xmin, _x.ymin, _x.xmax, _x.ymax, _x.confidence))
      _x = self.obj
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.distance
      buff.write(_get_struct_f().pack(_x))
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
      end += 20
      (_x.xmin, _x.ymin, _x.xmax, _x.ymax, _x.confidence,) = _get_struct_5f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.obj = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.obj = str[start:end]
      start = end
      end += 4
      (self.distance,) = _get_struct_f().unpack(str[start:end])
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
      buff.write(_get_struct_5f().pack(_x.xmin, _x.ymin, _x.xmax, _x.ymax, _x.confidence))
      _x = self.obj
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.distance
      buff.write(_get_struct_f().pack(_x))
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
      end += 20
      (_x.xmin, _x.ymin, _x.xmax, _x.ymax, _x.confidence,) = _get_struct_5f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.obj = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.obj = str[start:end]
      start = end
      end += 4
      (self.distance,) = _get_struct_f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_5f = None
def _get_struct_5f():
    global _struct_5f
    if _struct_5f is None:
        _struct_5f = struct.Struct("<5f")
    return _struct_5f
_struct_f = None
def _get_struct_f():
    global _struct_f
    if _struct_f is None:
        _struct_f = struct.Struct("<f")
    return _struct_f
