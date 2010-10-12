#!/usr/bin/env python

import xmlobject
import gentools

import re
import os.path
import sys

USAGE = "Usage: %s messages_xml_file.xml" % sys.argv[0]
H = "MESSAGES_H"

LENGTHS = {
    "uint8"     :   1,
    "int8"      :   1,
    "uint16"    :   2,
    "int16"     :   2,
    "uint32"    :   4,
    "int32"     :   4,
    "float"     :   4,
}
ARRAY_LENGTH = re.compile("^([a-z0-9]+)\[(\d{1,2})\]$")

def get_field_length(_type):
    #check if it is an array
    m = ARRAY_LENGTH.match(_type)
    if m:
        _type, _len = m.groups()
        return LENGTHS[_type] * int(_len)
    else:
        return LENGTHS[_type]

class Message:
    def __init__(self, m):
        self.name = m.name.upper()
        if int(m.id) <= 255:
            self.id = int(m.id)
        else:
            raise Exception("Message IDs must be <= 255")
        try:
            self.fields = m.field
        except:
            self.fields = []
        self.sizes = ["0"] + [str(get_field_length(f.type)) for f in self.fields]

    def print_id(self):
        print "#define MESSAGE_ID_%s %d" % (self.name, self.id)

    def print_send_function(self):
        print "#define MESSAGE_SEND_%s(" % self.name,
        print ", ".join([f.name for f in self.fields]), ") \\"
        print "{ \\"
        print "\tif (DownlinkCheckFreeSpace(%s)) { \\" % "+".join(self.sizes)
        print "\t\tDownlinkStartMessage(\"%s\", MESSAGE_ID_%s, %s) \\" % (self.name, self.name, "+".join(self.sizes))
        for f in self.fields:
            print "\t\t_Put%sByAddr((%s)) \\" % (f.type.title(), f.name)
        print "\t\tDownlinkEndMessage() \\"
        print "\t} else \\"
        print "\t\tDownlinkOverrun(); \\"
        print "}"
        print

    def print_accessor(self):
        offset = 0
        for f in self.fields:
            print "#define MESSAGE_%s_GET_FROM_BUFFER_%s(_payload)" % (self.name, f.name),
            if ARRAY_LENGTH.match(f.type):
                pass
            else:
                l = get_field_length(f.type)
                if l == 1:
                    print "(%s)(*((uint8_t*)_payload+%d))" % (f.type, offset)
                elif l == 2:
                    print "(%s)(*((uint8_t*)_payload+%d)|*((uint8_t*)_payload+%d+1)<<8)" % (f.type, offset, offset)
                elif l == 4:
                    if f.type == "float":
                        print "({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_payload+%d)|*((uint8_t*)_payload+%d+1)<<8|((uint32_t)*((uint8_t*)_payload+%d+2))<<16|((uint32_t)*((uint8_t*)_payload+%d+3))<<24); _f.f; })" % (offset, offset, offset, offset)
                    else:
                        print "(%s)(*((uint8_t*)_payload+%d)|*((uint8_t*)_payload+%d+1)<<8|((uint32_t)*((uint8_t*)_payload+%d+2))<<16|((uint32_t)*((uint8_t*)_payload+%d+3))<<24)" % (f.type, offset, offset, offset, offset)

if len(sys.argv) != 2:
    print USAGE
    sys.exit(1)
    
try:
    x = xmlobject.XMLFile(path=sys.argv[1])
    messages = [Message(m) for m in x.root.message]
except:
    import traceback
    traceback.print_exc()
    print "Invalid or missing xml\n  %s" % USAGE
    sys.exit(1)

gentools.print_header(H, generatedfrom=os.path.abspath(sys.argv[1]))

print "#define _Put1ByteByAddr(_byte) {	 \\"
print "\tuint8_t _x = *(_byte);		 \\"
print "\tDownlinkPutUint8(_x);	 \\"
print "}"
print "#define _Put2ByteByAddr(_byte) { \\"
print "\t_Put1ByteByAddr(_byte);	\\"
print "\t_Put1ByteByAddr((const uint8_t*)_byte+1);	\\"
print "}"
print "#define _Put4ByteByAddr(_byte) { \\"
print "\t_Put2ByteByAddr(_byte);	\\"
print "\t_Put2ByteByAddr((const uint8_t*)_byte+2);	\\"
print "}"
print "#define _PutInt8ByAddr(_x) _Put1ByteByAddr(_x)"
print "#define _PutUint8ByAddr(_x) _Put1ByteByAddr((const uint8_t*)_x)"
print "#define _PutInt16ByAddr(_x) _Put2ByteByAddr((const uint8_t*)_x)"
print "#define _PutUint16ByAddr(_x) _Put2ByteByAddr((const uint8_t*)_x)"
print "#define _PutInt32ByAddr(_x) _Put4ByteByAddr((const uint8_t*)_x)"
print "#define _PutUint32ByAddr(_x) _Put4ByteByAddr((const uint8_t*)_x)"
print "#define _PutFloatByAddr(_x) _Put4ByteByAddr((const uint8_t*)_x)"
print
for m in messages:
    m.print_id()
print
for m in messages:
    m.print_send_function()
print
for m in messages:
    m.print_accessor()

gentools.print_footer(H)



