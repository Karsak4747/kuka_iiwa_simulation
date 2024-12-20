/* pb_encode.c -- encode a protobuf using minimal resources
 *
 * 2011 Petteri Aimonen <jpa@kapsi.fi>
 */

#include "pb.h"
#include "pb_encode.h"

/* Use the GCC warn_unused_result attribute to check that all return values
 * are propagated correctly. On other compilers and gcc before 3.4.0 just
 * ignore the annotation.
 */
#if !defined(__GNUC__) || ( __GNUC__ < 3) || (__GNUC__ == 3 && __GNUC_MINOR__ < 4)
    #define checkreturn
#else
    #define checkreturn __attribute__((warn_unused_result))
#endif

/**************************************
 * Declarations internal to this file *
 **************************************/
typedef bool (*pb_encoder_t)(pb_ostream_t *stream, const pb_field_t *field, const void *src) checkreturn;

static bool checkreturn buf_write(pb_ostream_t *stream, const uint8_t *buf, size_t count);
static bool checkreturn encode_array(pb_ostream_t *stream, const pb_field_t *field, const void *pData, size_t count, pb_encoder_t func);
static bool checkreturn encode_field(pb_ostream_t *stream, const pb_field_t *field, const void *pData);
static bool checkreturn default_extension_encoder(pb_ostream_t *stream, const pb_extension_t *extension);
static bool checkreturn encode_extension_field(pb_ostream_t *stream, const pb_field_t *field, const void *pData);
static bool checkreturn pb_enc_varint(pb_ostream_t *stream, const pb_field_t *field, const void *src);
static bool checkreturn pb_enc_uvarint(pb_ostream_t *stream, const pb_field_t *field, const void *src);
static bool checkreturn pb_enc_svarint(pb_ostream_t *stream, const pb_field_t *field, const void *src);
static bool checkreturn pb_enc_fixed32(pb_ostream_t *stream, const pb_field_t *field, const void *src);
static bool checkreturn pb_enc_fixed64(pb_ostream_t *stream, const pb_field_t *field, const void *src);
static bool checkreturn pb_enc_bytes(pb_ostream_t *stream, const pb_field_t *field, const void *src);
static bool checkreturn pb_enc_string(pb_ostream_t *stream, const pb_field_t *field, const void *src);
static bool checkreturn pb_enc_submessage(pb_ostream_t *stream, const pb_field_t *field, const void *src);

/* --- Function pointers to field encoders ---
 * Order in the array must match pb_action_t LTYPE numbering.
 */
static const pb_encoder_t PB_ENCODERS[PB_LTYPES_COUNT] = {
    &pb_enc_varint,
    &pb_enc_uvarint,
    &pb_enc_svarint,
    &pb_enc_fixed32,
    &pb_enc_fixed64,

    &pb_enc_bytes,
    &pb_enc_string,
    &pb_enc_submessage,
    NULL /* extensions */
};

/*******************************
 * pb_ostream_t implementation *
 *******************************/

static bool checkreturn buf_write(pb_ostream_t *stream, const uint8_t *buf, size_t count)
{
    uint8_t *dest = (uint8_t*)stream->state;
    stream->state = dest + count;

    while (count--)
        *dest++ = *buf++;

    return true;
}

pb_ostream_t pb_ostream_from_buffer(uint8_t *buf, size_t bufsize)
{
    pb_ostream_t stream;
#ifdef PB_BUFFER_ONLY
    stream.callback = (void*)1; /* Just a marker value */
#else
    stream.callback = &buf_write;
#endif
    stream.state = buf;
    stream.max_size = bufsize;
    stream.bytes_written = 0;
#ifndef PB_NO_ERRMSG
    stream.errmsg = NULL;
#endif
    return stream;
}

bool checkreturn pb_write(pb_ostream_t *stream, const uint8_t *buf, size_t count)
{
    if (stream->callback != NULL)
    {
        if (stream->bytes_written + count > stream->max_size)
            PB_RETURN_ERROR(stream, "stream full");

#ifdef PB_BUFFER_ONLY
        if (!buf_write(stream, buf, count))
            PB_RETURN_ERROR(stream, "io error");
#else
        if (!stream->callback(stream, buf, count))
            PB_RETURN_ERROR(stream, "io error");
#endif
    }

    stream->bytes_written += count;
    return true;
}

/*************************
 * Encode a single field *
 *************************/

/* Encode a static array. Handles the size calculations and possible packing. */
static bool checkreturn encode_array(pb_ostream_t *stream, const pb_field_t *field,
                         const void *pData, size_t count, pb_encoder_t func)
{
    size_t i;
    const void *p;
    size_t size;

    if (count == 0)
        return true;

    if (PB_ATYPE(field->type) != PB_ATYPE_POINTER && count > field->array_size)
        PB_RETURN_ERROR(stream, "array max size exceeded");

    /* We always pack arrays if the datatype allows it. */
    if (PB_LTYPE(field->type) <= PB_LTYPE_LAST_PACKABLE)
    {
        if (!pb_encode_tag(stream, PB_WT_STRING, field->tag))
            return false;

        /* Determine the total size of packed array. */
        if (PB_LTYPE(field->type) == PB_LTYPE_FIXED32)
        {
            size = 4 * count;
        }
        else if (PB_LTYPE(field->type) == PB_LTYPE_FIXED64)
        {
            size = 8 * count;
        }
        else
        {
            pb_ostream_t sizestream = PB_OSTREAM_SIZING;
            p = pData;
            for (i = 0; i < count; i++)
            {
                if (!func(&sizestream, field, p))
                    return false;
                p = (const char*)p + field->data_size;
            }
            size = sizestream.bytes_written;
        }

        if (!pb_encode_varint(stream, (uint64_t)size))
            return false;

        if (stream->callback == NULL)
            return pb_write(stream, NULL, size); /* Just sizing.. */

        /* Write the data */
        p = pData;
        for (i = 0; i < count; i++)
        {
            if (!func(stream, field, p))
                return false;
            p = (const char*)p + field->data_size;
        }
    }
    else
    {
        p = pData;
        for (i = 0; i < count; i++)
        {
            if (!pb_encode_tag_for_field(stream, field))
                return false;

            /* Normally the data is stored directly in the array entries, but
             * for pointer-type string and bytes fields, the array entries are
             * actually pointers themselves also. So we have to dereference once
             * more to get to the actual data. */
            if (PB_ATYPE(field->type) == PB_ATYPE_POINTER &&
                (PB_LTYPE(field->type) == PB_LTYPE_STRING ||
                 PB_LTYPE(field->type) == PB_LTYPE_BYTES))
            {
                if (!func(stream, field, *(const void* const*)p))
                    return false;
            }
            else
            {
                if (!func(stream, field, p))
                    return false;
            }
            p = (const char*)p + field->data_size;
        }
    }

    return true;
}

/* Encode a field with static or pointer allocation, i.e. one whose data
 * is available to the encoder directly. */
static bool checkreturn encode_basic_field(pb_ostream_t *stream,
    const pb_field_t *field, const void *pData)
{
    pb_encoder_t func;
    const void *pSize;
    bool implicit_has = true;

    func = PB_ENCODERS[PB_LTYPE(field->type)];

    if (field->size_offset)
        pSize = (const char*)pData + field->size_offset;
    else
        pSize = &implicit_has;

    if (PB_ATYPE(field->type) == PB_ATYPE_POINTER)
    {
        /* pData is a pointer to the field, which contains pointer to
         * the data. If the 2nd pointer is NULL, it is interpreted as if
         * the has_field was false.
         */

        pData = *(const void* const*)pData;
        implicit_has = (pData != NULL);
    }

    switch (PB_HTYPE(field->type))
    {
        case PB_HTYPE_REQUIRED:
            if (!pData)
                PB_RETURN_ERROR(stream, "missing required field");
            if (!pb_encode_tag_for_field(stream, field))
                return false;
            if (!func(stream, field, pData))
                return false;
            break;

        case PB_HTYPE_OPTIONAL:
            /*
             * KUKA adjustment for VxWorks
             */

            if (*(const char*)pSize)
            {
                if (!pb_encode_tag_for_field(stream, field))
                    return false;

                if (!func(stream, field, pData))
                    return false;
            }
            break;

        case PB_HTYPE_REPEATED:
            if (!encode_array(stream, field, pData, *(const size_t*)pSize, func))
                return false;
            break;

        default:
            PB_RETURN_ERROR(stream, "invalid field type");
    }

    return true;
}

/* Encode a field with callback semantics. This means that a user function is
 * called to provide and encode the actual data. */
static bool checkreturn encode_callback_field(pb_ostream_t *stream,
    const pb_field_t *field, const void *pData)
{
    const pb_callback_t *callback = (const pb_callback_t*)pData;

#ifdef PB_OLD_CALLBACK_STYLE
    const void *arg = callback->arg;
#else
    void * const *arg = &(callback->arg);
#endif

    if (callback->funcs.encode != NULL)
    {
        if (!callback->funcs.encode(stream, field, arg))
            PB_RETURN_ERROR(stream, "callback error");
    }
    return true;
}

/* Encode a single field of any callback or static type. */
static bool checkreturn encode_field(pb_ostream_t *stream,
    const pb_field_t *field, const void *pData)
{
    switch (PB_ATYPE(field->type))
    {
        case PB_ATYPE_STATIC:
        case PB_ATYPE_POINTER:
            return encode_basic_field(stream, field, pData);

        case PB_ATYPE_CALLBACK:
            return encode_callback_field(stream, field, pData);

        default:
            PB_RETURN_ERROR(stream, "invalid field type");
    }
}

/* Default handler for extension fields. Expects to have a pb_field_t
 * pointer in the extension->type->arg field. */
static bool checkreturn default_extension_encoder(pb_ostream_t *stream,
    const pb_extension_t *extension)
{
    const pb_field_t *field = (const pb_field_t*)extension->type->arg;
    return encode_field(stream, field, extension->dest);
}

/* Walk through all the registered extensions and give them a chance
 * to encode themselves. */
static bool checkreturn encode_extension_field(pb_ostream_t *stream,
    const pb_field_t *field, const void *pData)
{
    const pb_extension_t *extension = *(const pb_extension_t* const *)pData;
    UNUSED(field);

    while (extension)
    {
        bool status;
        if (extension->type->encode)
            status = extension->type->encode(stream, extension);
        else
            status = default_extension_encoder(stream, extension);

        if (!status)
            return false;

        extension = extension->next;
    }

    return true;
}

/*********************
 * Encode all fields *
 *********************/

bool checkreturn pb_encode(pb_ostream_t *stream, const pb_field_t fields[], const void *src_struct)
{
    const pb_field_t *field = fields;
    const void *pData = src_struct;
    size_t prev_size = 0;

    while (field->tag != 0)
    {
        pData = (const char*)pData + prev_size + field->data_offset;
        if (PB_ATYPE(field->type) == PB_ATYPE_POINTER)
            prev_size = sizeof(const void*);
        else
            prev_size = field->data_size;

        /* Special case for static arrays */
        if (PB_ATYPE(field->type) == PB_ATYPE_STATIC &&
            PB_HTYPE(field->type) == PB_HTYPE_REPEATED)
        {
            prev_size *= field->array_size;
        }

        if (PB_LTYPE(field->type) == PB_LTYPE_EXTENSION)
        {
            /* Special case for the extension field placeholder */
            if (!encode_extension_field(stream, field, pData))
                return false;
        }
        else
        {
            /* Regular field */
            if (!encode_field(stream, field, pData))
                return false;
        }

        field++;
    }

    return true;
}

bool pb_encode_delimited(pb_ostream_t *stream, const pb_field_t fields[], const void *src_struct)
{
    return pb_encode_submessage(stream, fields, src_struct);
}

bool pb_get_encoded_size(size_t *size, const pb_field_t fields[], const void *src_struct)
{
    pb_ostream_t stream = PB_OSTREAM_SIZING;

    if (!pb_encode(&stream, fields, src_struct))
        return false;

    *size = stream.bytes_written;
    return true;
}

/********************
 * Helper functions *
 ********************/
bool checkreturn pb_encode_varint(pb_ostream_t *stream, uint64_t value)
{
    uint8_t buffer[10];
    size_t i = 0;

    if (value == 0)
        return pb_write(stream, (uint8_t*)&value, 1);

    while (value)
    {
        buffer[i] = (uint8_t)((value & 0x7F) | 0x80);
        value >>= 7;
        i++;
    }
    buffer[i-1] &= 0x7F; /* Unset top bit on last byte */

    return pb_write(stream, buffer, i);
}

bool checkreturn pb_encode_svarint(pb_ostream_t *stream, int64_t value)
{
    uint64_t zigzagged;
    if (value < 0)
        zigzagged = ~((uint64_t)value << 1);
    else
        zigzagged = (uint64_t)value << 1;

    return pb_encode_varint(stream, zigzagged);
}

bool checkreturn pb_encode_fixed32(pb_ostream_t *stream, const void *value)
{
    #ifdef __BIG_ENDIAN__
    const uint8_t *bytes = value;
    uint8_t lebytes[4];
    lebytes[0] = bytes[3];
    lebytes[1] = bytes[2];
    lebytes[2] = bytes[1];
    lebytes[3] = bytes[0];
    return pb_write(stream, lebytes, 4);
    #else
    return pb_write(stream, (const uint8_t*)value, 4);
    #endif
}

bool checkreturn pb_encode_fixed64(pb_ostream_t *stream, const void *value)
{
    #ifdef __BIG_ENDIAN__
    const uint8_t *bytes = value;
    uint8_t lebytes[8];
    lebytes[0] = bytes[7];
    lebytes[1] = bytes[6];
    lebytes[2] = bytes[5];
    lebytes[3] = bytes[4];
    lebytes[4] = bytes[3];
    lebytes[5] = bytes[2];
    lebytes[6] = bytes[1];
    lebytes[7] = bytes[0];
    return pb_write(stream, lebytes, 8);
    #else
    return pb_write(stream, (const uint8_t*)value, 8);
    #endif
}

bool checkreturn pb_encode_tag(pb_ostream_t *stream, pb_wire_type_t wiretype, uint32_t field_number)
{
    uint64_t tag = ((uint64_t)field_number << 3) | wiretype;
    return pb_encode_varint(stream, tag);
}

bool checkreturn pb_encode_tag_for_field(pb_ostream_t *stream, const pb_field_t *field)
{
    pb_wire_type_t wiretype;
    switch (PB_LTYPE(field->type))
    {
        case PB_LTYPE_VARINT:
        case PB_LTYPE_UVARINT:
        case PB_LTYPE_SVARINT:
            wiretype = PB_WT_VARINT;
            break;

        case PB_LTYPE_FIXED32:
            wiretype = PB_WT_32BIT;
            break;

        case PB_LTYPE_FIXED64:
            wiretype = PB_WT_64BIT;
            break;

        case PB_LTYPE_BYTES:
        case PB_LTYPE_STRING:
        case PB_LTYPE_SUBMESSAGE:
            wiretype = PB_WT_STRING;
            break;

        default:
            PB_RETURN_ERROR(stream, "invalid field type");
    }

    return pb_encode_tag(stream, wiretype, field->tag);
}

bool checkreturn pb_encode_string(pb_ostream_t *stream, const uint8_t *buffer, size_t size)
{
    if (!pb_encode_varint(stream, (uint64_t)size))
        return false;

    return pb_write(stream, buffer, size);
}

bool checkreturn pb_encode_submessage(pb_ostream_t *stream, const pb_field_t fields[], const void *src_struct)
{
    /* First calculate the message size using a non-writing substream. */
    pb_ostream_t substream = PB_OSTREAM_SIZING;
    size_t size;
    bool status;

    if (!pb_encode(&substream, fields, src_struct))
    {
#ifndef PB_NO_ERRMSG
        stream->errmsg = substream.errmsg;
#endif
        return false;
    }

    size = substream.bytes_written;

    if (!pb_encode_varint(stream, (uint64_t)size))
        return false;

    if (stream->callback == NULL)
        return pb_write(stream, NULL, size); /* Just sizing */

    if (stream->bytes_written + size > stream->max_size)
        PB_RETURN_ERROR(stream, "stream full");

    /* Use a substream to verify that a callback doesn't write more than
     * what it did the first time. */
    substream.callback = stream->callback;
    substream.state = stream->state;
    substream.max_size = size;
    substream.bytes_written = 0;
#ifndef PB_NO_ERRMSG
    substream.errmsg = NULL;
#endif

    status = pb_encode(&substream, fields, src_struct);

    stream->bytes_written += substream.bytes_written;
    stream->state = substream.state;
#ifndef PB_NO_ERRMSG
    stream->errmsg = substream.errmsg;
#endif

    if (substream.bytes_written != size)
        PB_RETURN_ERROR(stream, "submsg size changed");

    return status;
}

/* Field encoders */

static bool checkreturn pb_enc_varint(pb_ostream_t *stream, const pb_field_t *field, const void *src)
{
    int64_t value = 0;

    /* Cases 1 and 2 are for compilers that have smaller types for bool
     * or enums. */
    switch (field->data_size)
    {
        case 1: value = *(const int8_t*)src; break;
        case 2: value = *(const int16_t*)src; break;
        case 4: value = *(const int32_t*)src; break;
        case 8: value = *(const int64_t*)src; break;
        default: PB_RETURN_ERROR(stream, "invalid data_size");
    }

    return pb_encode_varint(stream, (uint64_t)value);
}

static bool checkreturn pb_enc_uvarint(pb_ostream_t *stream, const pb_field_t *field, const void *src)
{
    uint64_t value = 0;

    switch (field->data_size)
    {
        case 4: value = *(const uint32_t*)src; break;
        case 8: value = *(const uint64_t*)src; break;
        default: PB_RETURN_ERROR(stream, "invalid data_size");
    }

    return pb_encode_varint(stream, value);
}

static bool checkreturn pb_enc_svarint(pb_ostream_t *stream, const pb_field_t *field, const void *src)
{
    int64_t value = 0;

    switch (field->data_size)
    {
        case 4: value = *(const int32_t*)src; break;
        case 8: value = *(const int64_t*)src; break;
        default: PB_RETURN_ERROR(stream, "invalid data_size");
    }

    return pb_encode_svarint(stream, value);
}

static bool checkreturn pb_enc_fixed64(pb_ostream_t *stream, const pb_field_t *field, const void *src)
{
    UNUSED(field);
    return pb_encode_fixed64(stream, src);
}

static bool checkreturn pb_enc_fixed32(pb_ostream_t *stream, const pb_field_t *field, const void *src)
{
    UNUSED(field);
    return pb_encode_fixed32(stream, src);
}

static bool checkreturn pb_enc_bytes(pb_ostream_t *stream, const pb_field_t *field, const void *src)
{
    const pb_bytes_array_t *bytes = (const pb_bytes_array_t*)src;

    if (src == NULL)
    {
        /* Threat null pointer as an empty bytes field */
        return pb_encode_string(stream, NULL, 0);
    }

    if (PB_ATYPE(field->type) == PB_ATYPE_STATIC &&
        PB_BYTES_ARRAY_T_ALLOCSIZE(bytes->size) > field->data_size)
    {
        PB_RETURN_ERROR(stream, "bytes size exceeded");
    }

    return pb_encode_string(stream, bytes->bytes, bytes->size);
}

static bool checkreturn pb_enc_string(pb_ostream_t *stream, const pb_field_t *field, const void *src)
{
    /* strnlen() is not always available, so just use a loop */
    size_t size = 0;
    size_t max_size = field->data_size;
    const char *p = (const char*)src;

    if (PB_ATYPE(field->type) == PB_ATYPE_POINTER)
        max_size = (size_t)-1;

    if (src == NULL)
    {
        size = 0; /* Threat null pointer as an empty string */
    }
    else
    {
        while (size < max_size && *p != '\0')
        {
            size++;
            p++;
        }
    }

    return pb_encode_string(stream, (const uint8_t*)src, size);
}

static bool checkreturn pb_enc_submessage(pb_ostream_t *stream, const pb_field_t *field, const void *src)
{
    if (field->ptr == NULL)
        PB_RETURN_ERROR(stream, "invalid field descriptor");

    return pb_encode_submessage(stream, (const pb_field_t*)field->ptr, src);
}
