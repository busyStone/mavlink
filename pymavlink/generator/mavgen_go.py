#!/usr/bin/env python
'''
parse a MAVLink protocol XML file and generate a Go implementation

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''

import sys, textwrap, os, time
import mavparse, mavtemplate

t = mavtemplate.MAVTemplate()

def generate_version_h(directory, xml):
    '''generate version.go'''
    f = open(os.path.join(directory, "version.go"), mode='w')
    t.write(f,'''
package mavlink

//
// MAVLink comm protocol built from ${basename}.xml
// http://pixhawk.ethz.ch/software/mavlink
//

const (
MAVLINK_BUILD_DATE = "${parse_time}"
MAVLINK_WIRE_PROTOCOL_VERSION = "${wire_protocol_version}"
MAVLINK_MAX_DIALECT_PAYLOAD_SIZE = ${largest_payload} 
MAVLINK_VERSION = ${version}
)

''', xml)
    f.close()

def generate_mavlink_h(directory, xml):
    '''generate mavlink.go'''
    f = open(os.path.join(directory, "mavlink.go"), mode='w')
    t.write(f,'''
package mavlink

// 
// MAVLink comm protocol built from ${basename}.xml
// http://pixhawk.ethz.ch/software/mavlink
//

import (
    "bytes"
    "encoding/binary"
    "io"
    "time"
)

const (
MAVLINK_BIG_ENDIAN = 0
MAVLINK_LITTLE_ENDIAN = 1
MAVLINK_STX = ${protocol_marker}
MAVLINK_ENDIAN = ${mavlink_endian}
MAVLINK_ALIGNED_FIELDS = ${aligned_fields_define}
MAVLINK_CRC_EXTRA = ${crc_extra_define}
X25_INIT_CRC = 0xffff
X25_VALIDATE_CRC = 0xf0b8
)

var sequence uint16 = 0

func generateSequence() uint8 {
        sequence = (sequence + 1) % 256
        return uint8(sequence)
}

// The MAVLinkMessage interface is implemented by MAVLink messages 
type MAVLinkMessage interface {
        Id() uint8
        Len() uint8
        Crc() uint8
        Pack() []byte
        Decode([]byte)
}

// A MAVLinkPacket represents a raw packet received from a micro air vehicle
type MAVLinkPacket struct {
        Protocol    uint8
        Length      uint8
        Sequence    uint8
        SystemID    uint8
        ComponentID uint8
        MessageID   uint8
        Data        []uint8
        Checksum    uint16
}

// ReadMAVLinkPacket reads an io.Reader for a new packet and returns a new MAVLink packet 
// or returns the error received by the io.Reader
func ReadMAVLinkPacket(r io.Reader) (*MAVLinkPacket, error) {
        for {
                header, err := read(r, 1)
                if err != nil {
                        return nil, err
                }
                if header[0] == 254 {
                        length, err := read(r, 1)
                        if err != nil {
                                return nil, err
                        } else if length[0] > 250 {
                                continue
                        }
                        m := &MAVLinkPacket{}
                        data, err := read(r, int(length[0])+7)
                        if err != nil {
                                return nil, err
                        }
                        data = append([]byte{header[0], length[0]}, data...)
                        m.Decode(data)
                        return m, nil
                }
        }
}

// CraftMAVLinkPacket returns a new MAVLinkPacket from a MAVLinkMessage
func CraftMAVLinkPacket(SystemID uint8, ComponentID uint8, Message MAVLinkMessage) *MAVLinkPacket {
        return NewMAVLinkPacket(
                0xFE,
                Message.Len(),
                generateSequence(),
                SystemID,
                ComponentID,
                Message.Id(),
                Message.Pack(),
        )
}

// NewMAVLinkPacket returns a new MAVLinkPacket
func NewMAVLinkPacket(Protocol uint8, Length uint8, Sequence uint8, SystemID uint8, ComponentID uint8, MessageID uint8, Data []uint8) *MAVLinkPacket {
        m := &MAVLinkPacket{
                Protocol:    Protocol,
                Length:      Length,
                Sequence:    Sequence,
                SystemID:    SystemID,
                ComponentID: ComponentID,
                MessageID:   MessageID,
                Data:        Data,
        }
        m.Checksum = crcCalculate(m)
        return m
}

// MAVLinkMessage returns the decoded MAVLinkMessage from the MAVLinkPacket
// or returns an error generated from the MAVLinkMessage
func (m *MAVLinkPacket) MAVLinkMessage() (MAVLinkMessage, error) {
        return NewMAVLinkMessage(m.MessageID, m.Data)
}

// Pack returns a packed byte array which represents the MAVLinkPacket
func (m *MAVLinkPacket) Pack() []byte {
        data := new(bytes.Buffer)
        binary.Write(data, binary.LittleEndian, m.Protocol)
        binary.Write(data, binary.LittleEndian, m.Length)
        binary.Write(data, binary.LittleEndian, m.Sequence)
        binary.Write(data, binary.LittleEndian, m.SystemID)
        binary.Write(data, binary.LittleEndian, m.ComponentID)
        binary.Write(data, binary.LittleEndian, m.MessageID)
        data.Write(m.Data)
        binary.Write(data, binary.LittleEndian, m.Checksum)
        return data.Bytes()
}

// Decode accepts a packed byte array and populates the fields of the MAVLinkPacket
func (m *MAVLinkPacket) Decode(buf []byte) {
        m.Protocol = buf[0]
        m.Length = buf[1]
        m.Sequence = buf[2]
        m.SystemID = buf[3]
        m.ComponentID = buf[4]
        m.MessageID = buf[5]
        m.Data = buf[6 : 6+int(m.Length)]
        checksum := buf[7+int(m.Length):]
        m.Checksum = uint16(checksum[1])<<8 | uint16(checksum[0])
}

func read(r io.Reader, length int) ([]byte, error) {
	buf := []byte{}
	for length > 0 {
		tmp := make([]byte, length)
		i, err := r.Read(tmp[:])
		if err != nil {
			return nil, err
		} else {
			length -= i
			buf = append(buf, tmp...)
			if length != i {
				<-time.After(1 * time.Millisecond)
			} else {
				break
			}
		}
	}
	return buf, nil
}

// 
// Accumulate the X.25 CRC by adding one char at a time.
// 
// The checksum function adds the hash of one char at a time to the
// 16 bit checksum (uint16).
// 
// data to hash
// crcAccum the already accumulated checksum
//
func crcAccumulate(data uint8, crcAccum uint16) uint16 {
        /*Accumulate one byte of data into the CRC*/
        var tmp uint8

        tmp = data ^ (uint8)(crcAccum&0xff)
        tmp ^= (tmp << 4)
        crcAccum = (uint16(crcAccum) >> 8) ^ (uint16(tmp) << 8) ^ (uint16(tmp) << 3) ^ (uint16(tmp) >> 4)
        return crcAccum
}

//
// Initiliaze the buffer for the X.25 CRC
//
func crcInit() uint16 {
        return X25_INIT_CRC
}

//
// Calculates the X.25 checksum on a byte buffer
//
// return the checksum over the buffer bytes
//
func crcCalculate(m *MAVLinkPacket) uint16 {
        crc := crcInit()

        for _, v := range m.Pack()[1 : m.Length+6] {
                crc = crcAccumulate(v, crc)
        }
        message, _ := m.MAVLinkMessage()
        crc = crcAccumulate(message.Crc(), crc)
        return crc
}

''', xml)
    f.close()

def generate_main_h(directory, xml):
    '''generate main header per XML file'''
    f = open(os.path.join(directory, xml.basename + ".go"), mode='w')
    t.write(f, '''
package mavlink

// 
// MAVLink comm protocol generated from ${basename}.xml
// http://qgroundcontrol.org/mavlink/
// 
import (
  "bytes"
  "encoding/binary"
  "errors"
  "fmt"
)

var messages = map[uint8]MAVLinkMessage{
    ${{message:${id}: &${name_camel_case}{},
    }}
  }

// NewMAVLinkMessage returns a new MAVLinkMessage or an error if it encounters an unknown Message ID
func NewMAVLinkMessage(msgid uint8, data []byte) (MAVLinkMessage, error) {
  message := messages[msgid]
  if message != nil {
    message.Decode(data)
    return message, nil
  }
  return nil, errors.New(fmt.Sprintf("Unknown Message ID: %v", msgid))
}

${{enum:
//
// ${name}
/*${description}*/
//
const (
${{entry:${name}=${value} // ${description} | ${{param:${description} | }}
}}
)
}}

''', xml)

    f.close()
             

def generate_message_h(directory, m):
    '''generate per-message header for a XML file'''
    f = open(os.path.join(directory, m.basename + ".go"), mode='a')
    t.write(f, '''
//
// MESSAGE ${name}
//
// MAVLINK_MSG_ID_${name} ${id}
//
// MAVLINK_MSG_ID_${name}_LEN ${wire_length}
//
// MAVLINK_MSG_ID_${name}_CRC ${crc_extra}
//
//
type ${name_camel_case} struct {
${{ordered_fields: ${name_upper} ${array_suffix}${type}  // ${description}
}}
} 

// New${name_camel_case} returns a new ${name_camel_case} 
func New${name_camel_case}(${{ordered_fields: ${name_upper} ${array_suffix}${type},}}) *${name_camel_case} {
  m := ${name_camel_case}{}
  ${{ordered_fields: m.${name_upper} = ${name_upper}
}}
  return &m
}

// Id returns the ${name_camel_case} Message ID 
func (*${name_camel_case}) Id() uint8 {
    return ${id}
}

// Len returns the ${name_camel_case} Message Length
func (*${name_camel_case}) Len() uint8 {
    return ${wire_length}
}

// Crc returns the ${name_camel_case} Message CRC
func (*${name_camel_case}) Crc() uint8 {
    return ${crc_extra}
}

// Pack returns a packed byte array which represents a ${name_camel_case} payload
func (m *${name_camel_case}) Pack() []byte {
  data := new(bytes.Buffer)
  ${{ordered_fields: binary.Write(data, binary.LittleEndian, m.${name_upper})
}}
  return data.Bytes()
}

// Decode accepts a packed byte array and populates the fields of the ${name_camel_case}
func (m *${name_camel_case}) Decode(buf []byte) {
  data := bytes.NewBuffer(buf)
  ${{ordered_fields: binary.Read(data, binary.LittleEndian, &m.${name_upper})
}}
}

const (
${{array_fields:MAVLINK_MSG_${msg_name}_FIELD_${name}_LEN = ${array_length}
}}
)
''', m)
    f.close()

def copy_fixed_headers(directory, xml):
    '''copy the fixed protocol headers to the target directory'''
    import shutil
    hlist = [ 'protocol.h', 'mavlink_helpers.h', 'mavlink_types.h', 'checksum.h', 'mavlink_conversions.h', 'mavlink_protobuf_manager.hpp' ]
    basepath = os.path.dirname(os.path.realpath(__file__))
    srcpath = os.path.join(basepath, 'Go/include_v%s' % xml.wire_protocol_version)
    print("Copying fixed headers")
    for h in hlist:
        if (not ((h == 'mavlink_protobuf_manager.hpp' or h == 'mavlink_conversions.h') and xml.wire_protocol_version == '0.9')):
           src = os.path.realpath(os.path.join(srcpath, h))
           dest = os.path.realpath(os.path.join(directory, h))
           if src == dest:
               continue
           shutil.copy(src, dest)
        
def copy_fixed_sources(directory, xml):
    # XXX This is a hack - to be removed
    import shutil
    basepath = os.path.dirname(os.path.realpath(__file__))
    srcpath = os.path.join(basepath, 'Go/src_v%s' % xml.wire_protocol_version)
    if (xml.basename == 'pixhawk' and xml.wire_protocol_version == '1.0'):
        print("Copying fixed sources")
        src = os.path.realpath(os.path.join(srcpath, 'pixhawk/pixhawk.pb.cc'))
        dest = os.path.realpath(os.path.join(directory, '../../../share/mavlink/src/v%s/pixhawk/pixhawk.pb.cc' % xml.wire_protocol_version))
        destdir = os.path.realpath(os.path.join(directory, '../../../share/mavlink/src/v%s/pixhawk' % xml.wire_protocol_version))
        try:
           os.makedirs(destdir)
        except:
           print("Not re-creating directory")
        shutil.copy(src, dest)
        print("Copied to"),
        print(dest)

class mav_include(object):
    def __init__(self, base):
        self.base = base

def generate_one(basename, xml):
    '''generate headers for one XML file'''

    directory = os.path.join(basename, xml.basename)

    print("Generating Go implementation in directory %s" % directory)
    mavparse.mkdir_p(directory)

    if xml.little_endian:
        xml.mavlink_endian = "MAVLINK_LITTLE_ENDIAN"
    else:
        xml.mavlink_endian = "MAVLINK_BIG_ENDIAN"

    if xml.crc_extra:
        xml.crc_extra_define = "1"
    else:
        xml.crc_extra_define = "0"

    if xml.sort_fields:
        xml.aligned_fields_define = "1"
    else:
        xml.aligned_fields_define = "0"

    # work out the included headers
    xml.include_list = []
    for i in xml.include:
        base = i[:-4]
        xml.include_list.append(mav_include(base))

    # form message lengths array
    xml.message_lengths_array = ''
    for mlen in xml.message_lengths:
        xml.message_lengths_array += '%u, ' % mlen
    xml.message_lengths_array = xml.message_lengths_array[:-2]

    # and message CRCs array
    xml.message_crcs_array = ''
    for crc in xml.message_crcs:
        xml.message_crcs_array += '%u, ' % crc
    xml.message_crcs_array = xml.message_crcs_array[:-2]

    # form message info array
    xml.message_info_array = ''
    for name in xml.message_names:
        if name is not None:
            xml.message_info_array += 'MAVLINK_MESSAGE_INFO_%s, ' % name
        else:
            # Several C compilers don't accept {NULL} for
            # multi-dimensional arrays and structs
            # feed the compiler a "filled" empty message
            xml.message_info_array += '{"EMPTY",0,{{"","",MAVLINK_TYPE_CHAR,0,0,0}}}, '
    xml.message_info_array = xml.message_info_array[:-2]

    # add some extra field attributes for convenience with arrays
    for m in xml.message:
        m.msg_name = m.name
        if xml.crc_extra:
            m.crc_extra_arg = ", %s" % m.crc_extra
        else:
            m.crc_extra_arg = ""
        for f in m.fields:
            if f.print_format is None:
                f.c_print_format = 'NULL'
            else:
                f.c_print_format = '"%s"' % f.print_format
            if f.array_length != 0:
                f.array_suffix = '[%u]' % f.array_length
                f.array_prefix = '*'
                f.array_tag = '_array'
                f.array_arg = ', %u' % f.array_length
                f.array_return_arg = '%s, %u, ' % (f.name, f.array_length)
                f.array_const = 'const '
                f.decode_left = ''
                f.decode_right = ', %s->%s' % (m.name_lower, f.name)
                f.return_type = 'uint16_t'
                f.get_arg = ', %s *%s' % (f.type, f.name)
                if f.type == 'char':
                    f.c_test_value = '"%s"' % f.test_value
                else:
                    test_strings = []
                    for v in f.test_value:
                        test_strings.append(str(v))
                    f.c_test_value = '{ %s }' % ', '.join(test_strings)
            else:
                f.array_suffix = ''
                f.array_prefix = ''
                f.array_tag = ''
                f.array_arg = ''
                f.array_return_arg = ''
                f.array_const = ''
                f.decode_left = "%s->%s = " % (m.name_lower, f.name)
                f.decode_right = ''
                f.get_arg = ''
                f.return_type = f.type
                if f.type == 'char':
                    f.c_test_value = "'%s'" % f.test_value
                elif f.type == 'uint64_t':
                    f.c_test_value = "%sULL" % f.test_value                    
                elif f.type == 'int64_t':
                    f.c_test_value = "%sLL" % f.test_value                    
                else:
                    f.c_test_value = f.test_value
            f.type = to_go_type(f.type)

    # cope with uint8_t_mavlink_version
    for m in xml.message:
        m.name_camel_case = to_camel_case(m.name)
        m.arg_fields = []
        m.array_fields = []
        m.scalar_fields = []
        for f in m.ordered_fields:
            if f.array_length != 0:
                m.array_fields.append(f)
            else:
                m.scalar_fields.append(f)
        for f in m.fields:
            if not f.omit_arg:
                m.arg_fields.append(f)
                f.putname = f.name
            else:
                f.putname = f.const_value

    generate_mavlink_h(directory, xml)
    generate_version_h(directory, xml)
    generate_main_h(directory, xml)
    for m in xml.message:
        m.basename = xml.basename
        generate_message_h(directory, m)
    #generate_testsuite_h(directory, xml)


def to_camel_case(snake_str):
    components = snake_str.split('_')
    # We capitalize the first letter of each component except the first one
    # with the 'title' method and join them together.
    return "".join(x.title() for x in components)

def to_go_type(field):
    '''return the Golang type'''
    map = {
        'float'    : 'float32',
        'double'   : 'float64',
        'char'     : 'uint8',
        'int8_t'   : 'int8',
        'uint8_t'  : 'uint8',
        'uint8_t_mavlink_version'  : 'uint8',
        'int16_t'  : 'int16',
        'uint16_t' : 'uint16',
        'int32_t'  : 'int32',
        'uint32_t' : 'uint32',
        'int64_t'  : 'int64',
        'uint64_t' : 'uint64',
        }

    return map[field]

def generate(basename, xml_list):
    '''generate complete MAVLink Go implemenation'''

    #for x in xml_list:
#   x.message.extend(x.message)
#       x.enum.extend(x.enum)
#        filelist.append(os.path.basename(x.filename))
#        print("x.filename " + x.filename)

    #for xml in xml_list:
    msgs = []
    enums = []
    for x in xml_list:
        msgs.extend(x.message)
        enums.extend(x.enum)

    xml_list[0].message = msgs
    xml_list[0].enum = enums

    generate_one(basename, xml_list[0])
    copy_fixed_headers(basename, xml_list[0])
    #copy_fixed_sources(basename, xml_list[0])
