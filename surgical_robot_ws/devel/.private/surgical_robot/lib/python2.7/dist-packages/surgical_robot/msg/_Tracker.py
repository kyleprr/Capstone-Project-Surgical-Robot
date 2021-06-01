# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from surgical_robot/Tracker.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Tracker(genpy.Message):
  _md5sum = "b283d7d9a91916dac4010d28ee78ee60"
  _type = "surgical_robot/Tracker"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# message type to describe the tracking information of the blocks
# to be published as a topic

float64 x  # x coordinate in the world
float64 y  # y coordinate in the world
float64 z  # z coordinate in the world
float64 error_x
float64 error_y
float64 error_z
bool flag1
bool flag2
bool flag3
"""
  __slots__ = ['x','y','z','error_x','error_y','error_z','flag1','flag2','flag3']
  _slot_types = ['float64','float64','float64','float64','float64','float64','bool','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       x,y,z,error_x,error_y,error_z,flag1,flag2,flag3

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Tracker, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
      if self.error_x is None:
        self.error_x = 0.
      if self.error_y is None:
        self.error_y = 0.
      if self.error_z is None:
        self.error_z = 0.
      if self.flag1 is None:
        self.flag1 = False
      if self.flag2 is None:
        self.flag2 = False
      if self.flag3 is None:
        self.flag3 = False
    else:
      self.x = 0.
      self.y = 0.
      self.z = 0.
      self.error_x = 0.
      self.error_y = 0.
      self.error_z = 0.
      self.flag1 = False
      self.flag2 = False
      self.flag3 = False

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
      buff.write(_get_struct_6d3B().pack(_x.x, _x.y, _x.z, _x.error_x, _x.error_y, _x.error_z, _x.flag1, _x.flag2, _x.flag3))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 51
      (_x.x, _x.y, _x.z, _x.error_x, _x.error_y, _x.error_z, _x.flag1, _x.flag2, _x.flag3,) = _get_struct_6d3B().unpack(str[start:end])
      self.flag1 = bool(self.flag1)
      self.flag2 = bool(self.flag2)
      self.flag3 = bool(self.flag3)
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
      buff.write(_get_struct_6d3B().pack(_x.x, _x.y, _x.z, _x.error_x, _x.error_y, _x.error_z, _x.flag1, _x.flag2, _x.flag3))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 51
      (_x.x, _x.y, _x.z, _x.error_x, _x.error_y, _x.error_z, _x.flag1, _x.flag2, _x.flag3,) = _get_struct_6d3B().unpack(str[start:end])
      self.flag1 = bool(self.flag1)
      self.flag2 = bool(self.flag2)
      self.flag3 = bool(self.flag3)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6d3B = None
def _get_struct_6d3B():
    global _struct_6d3B
    if _struct_6d3B is None:
        _struct_6d3B = struct.Struct("<6d3B")
    return _struct_6d3B
