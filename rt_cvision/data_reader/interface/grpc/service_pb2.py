# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: service.proto
# Protobuf Python Version: 4.25.1
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\rservice.proto\x12\x10\x64\x61ta_acquisition\"\"\n\x12ProcessDataRequest\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\t\"%\n\x13ProcessDataResponse\x12\x0e\n\x06result\x18\x01 \x01(\t2m\n\rComputingUnit\x12\\\n\x0bProcessData\x12$.data_acquisition.ProcessDataRequest\x1a%.data_acquisition.ProcessDataResponse\"\x00\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'service_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_PROCESSDATAREQUEST']._serialized_start=35
  _globals['_PROCESSDATAREQUEST']._serialized_end=69
  _globals['_PROCESSDATARESPONSE']._serialized_start=71
  _globals['_PROCESSDATARESPONSE']._serialized_end=108
  _globals['_COMPUTINGUNIT']._serialized_start=110
  _globals['_COMPUTINGUNIT']._serialized_end=219
# @@protoc_insertion_point(module_scope)