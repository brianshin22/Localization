"""autogenerated by genpy from localize/LaneMeasure.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class LaneMeasure(genpy.Message):
  _md5sum = "c83232960a83d0a1fdaef90d75536fce"
  _type = "localize/LaneMeasure"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 a0
float64 a1
float64 a2
float64 a3
int32 quality


"""
  __slots__ = ['a0','a1','a2','a3','quality']
  _slot_types = ['float64','float64','float64','float64','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       a0,a1,a2,a3,quality

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(LaneMeasure, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.a0 is None:
        self.a0 = 0.
      if self.a1 is None:
        self.a1 = 0.
      if self.a2 is None:
        self.a2 = 0.
      if self.a3 is None:
        self.a3 = 0.
      if self.quality is None:
        self.quality = 0
    else:
      self.a0 = 0.
      self.a1 = 0.
      self.a2 = 0.
      self.a3 = 0.
      self.quality = 0

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
      buff.write(_struct_4di.pack(_x.a0, _x.a1, _x.a2, _x.a3, _x.quality))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 36
      (_x.a0, _x.a1, _x.a2, _x.a3, _x.quality,) = _struct_4di.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_4di.pack(_x.a0, _x.a1, _x.a2, _x.a3, _x.quality))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 36
      (_x.a0, _x.a1, _x.a2, _x.a3, _x.quality,) = _struct_4di.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4di = struct.Struct("<4di")
