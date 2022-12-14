# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: time_step.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0ftime_step.proto\"f\n\x08TimeStep\x12\x0c\n\x04time\x18\x01 \x01(\x04\x12\x16\n\x06robots\x18\x02 \x03(\x0b\x32\x06.Robot\x12\x14\n\x05tasks\x18\x03 \x03(\x0b\x32\x05.Task\x12\x13\n\x06points\x18\x04 \x01(\x04H\x00\x88\x01\x01\x42\t\n\x07_points\" \n\x08Position\x12\t\n\x01x\x18\x01 \x01(\x01\x12\t\n\x01y\x18\x02 \x01(\x01\"\x93\x02\n\x05Robot\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x0e\n\x06teamID\x18\x02 \x01(\x04\x12\x1b\n\x05state\x18\x03 \x01(\x0e\x32\x0c.Robot.State\x12\x1b\n\x08position\x18\x04 \x01(\x0b\x32\t.Position\x12\x16\n\ttotalSent\x18\x05 \x01(\x04H\x00\x88\x01\x01\x12\x1a\n\rtotalReceived\x18\x06 \x01(\x04H\x01\x88\x01\x01\x12\x13\n\x06\x61\x63tion\x18\x07 \x01(\tH\x02\x88\x01\x01\">\n\x05State\x12\x0c\n\x08\x46OLLOWER\x10\x00\x12\n\n\x06LEADER\x10\x01\x12\r\n\tCONNECTOR\x10\x02\x12\x0c\n\x08TRAVELER\x10\x03\x42\x0c\n\n_totalSentB\x10\n\x0e_totalReceivedB\t\n\x07_action\"p\n\x04Task\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x0e\n\x06\x64\x65mand\x18\x02 \x01(\x04\x12\x16\n\x0erequiredRobots\x18\x03 \x01(\x04\x12\x15\n\rcurrentRobots\x18\x04 \x01(\x04\x12\x1b\n\x08position\x18\x05 \x01(\x0b\x32\t.Positionb\x06proto3')



_TIMESTEP = DESCRIPTOR.message_types_by_name['TimeStep']
_POSITION = DESCRIPTOR.message_types_by_name['Position']
_ROBOT = DESCRIPTOR.message_types_by_name['Robot']
_TASK = DESCRIPTOR.message_types_by_name['Task']
_ROBOT_STATE = _ROBOT.enum_types_by_name['State']
TimeStep = _reflection.GeneratedProtocolMessageType('TimeStep', (_message.Message,), {
  'DESCRIPTOR' : _TIMESTEP,
  '__module__' : 'time_step_pb2'
  # @@protoc_insertion_point(class_scope:TimeStep)
  })
_sym_db.RegisterMessage(TimeStep)

Position = _reflection.GeneratedProtocolMessageType('Position', (_message.Message,), {
  'DESCRIPTOR' : _POSITION,
  '__module__' : 'time_step_pb2'
  # @@protoc_insertion_point(class_scope:Position)
  })
_sym_db.RegisterMessage(Position)

Robot = _reflection.GeneratedProtocolMessageType('Robot', (_message.Message,), {
  'DESCRIPTOR' : _ROBOT,
  '__module__' : 'time_step_pb2'
  # @@protoc_insertion_point(class_scope:Robot)
  })
_sym_db.RegisterMessage(Robot)

Task = _reflection.GeneratedProtocolMessageType('Task', (_message.Message,), {
  'DESCRIPTOR' : _TASK,
  '__module__' : 'time_step_pb2'
  # @@protoc_insertion_point(class_scope:Task)
  })
_sym_db.RegisterMessage(Task)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _TIMESTEP._serialized_start=19
  _TIMESTEP._serialized_end=121
  _POSITION._serialized_start=123
  _POSITION._serialized_end=155
  _ROBOT._serialized_start=158
  _ROBOT._serialized_end=433
  _ROBOT_STATE._serialized_start=328
  _ROBOT_STATE._serialized_end=390
  _TASK._serialized_start=435
  _TASK._serialized_end=547
# @@protoc_insertion_point(module_scope)
