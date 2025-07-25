// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file MPU6050MSG.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace {
char dummy;
}  // namespace
#endif  // _WIN32

#include "MPU6050MSG.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

MPU6050MSG::MPU6050MSG()
{
    // m_ax com.eprosima.idl.parser.typecode.PrimitiveTypeCode@3911c2a7
    m_ax = 0;
    // m_ay com.eprosima.idl.parser.typecode.PrimitiveTypeCode@4ac3c60d
    m_ay = 0;
    // m_az com.eprosima.idl.parser.typecode.PrimitiveTypeCode@4facf68f
    m_az = 0;
    // m_gx com.eprosima.idl.parser.typecode.PrimitiveTypeCode@76508ed1
    m_gx = 0;
    // m_gy com.eprosima.idl.parser.typecode.PrimitiveTypeCode@41e36e46
    m_gy = 0;
    // m_gz com.eprosima.idl.parser.typecode.PrimitiveTypeCode@15c43bd9
    m_gz = 0;
    // m_timestamp com.eprosima.idl.parser.typecode.PrimitiveTypeCode@3d74bf60
    m_timestamp = 0;

}

MPU6050MSG::~MPU6050MSG()
{






}

MPU6050MSG::MPU6050MSG(
        const MPU6050MSG& x)
{
    m_ax = x.m_ax;
    m_ay = x.m_ay;
    m_az = x.m_az;
    m_gx = x.m_gx;
    m_gy = x.m_gy;
    m_gz = x.m_gz;
    m_timestamp = x.m_timestamp;
}

MPU6050MSG::MPU6050MSG(
        MPU6050MSG&& x)
{
    m_ax = x.m_ax;
    m_ay = x.m_ay;
    m_az = x.m_az;
    m_gx = x.m_gx;
    m_gy = x.m_gy;
    m_gz = x.m_gz;
    m_timestamp = x.m_timestamp;
}

MPU6050MSG& MPU6050MSG::operator =(
        const MPU6050MSG& x)
{

    m_ax = x.m_ax;
    m_ay = x.m_ay;
    m_az = x.m_az;
    m_gx = x.m_gx;
    m_gy = x.m_gy;
    m_gz = x.m_gz;
    m_timestamp = x.m_timestamp;

    return *this;
}

MPU6050MSG& MPU6050MSG::operator =(
        MPU6050MSG&& x)
{

    m_ax = x.m_ax;
    m_ay = x.m_ay;
    m_az = x.m_az;
    m_gx = x.m_gx;
    m_gy = x.m_gy;
    m_gz = x.m_gz;
    m_timestamp = x.m_timestamp;

    return *this;
}

bool MPU6050MSG::operator ==(
        const MPU6050MSG& x) const
{

    return (m_ax == x.m_ax && m_ay == x.m_ay && m_az == x.m_az && m_gx == x.m_gx && m_gy == x.m_gy && m_gz == x.m_gz && m_timestamp == x.m_timestamp);
}

bool MPU6050MSG::operator !=(
        const MPU6050MSG& x) const
{
    return !(*this == x);
}

size_t MPU6050MSG::getMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

size_t MPU6050MSG::getCdrSerializedSize(
        const MPU6050MSG& data,
        size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

void MPU6050MSG::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{

    scdr << m_ax;
    scdr << m_ay;
    scdr << m_az;
    scdr << m_gx;
    scdr << m_gy;
    scdr << m_gz;
    scdr << m_timestamp;

}

void MPU6050MSG::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{

    dcdr >> m_ax;
    dcdr >> m_ay;
    dcdr >> m_az;
    dcdr >> m_gx;
    dcdr >> m_gy;
    dcdr >> m_gz;
    dcdr >> m_timestamp;
}

/*!
 * @brief This function sets a value in member ax
 * @param _ax New value for member ax
 */
void MPU6050MSG::ax(
        int16_t _ax)
{
    m_ax = _ax;
}

/*!
 * @brief This function returns the value of member ax
 * @return Value of member ax
 */
int16_t MPU6050MSG::ax() const
{
    return m_ax;
}

/*!
 * @brief This function returns a reference to member ax
 * @return Reference to member ax
 */
int16_t& MPU6050MSG::ax()
{
    return m_ax;
}

/*!
 * @brief This function sets a value in member ay
 * @param _ay New value for member ay
 */
void MPU6050MSG::ay(
        int16_t _ay)
{
    m_ay = _ay;
}

/*!
 * @brief This function returns the value of member ay
 * @return Value of member ay
 */
int16_t MPU6050MSG::ay() const
{
    return m_ay;
}

/*!
 * @brief This function returns a reference to member ay
 * @return Reference to member ay
 */
int16_t& MPU6050MSG::ay()
{
    return m_ay;
}

/*!
 * @brief This function sets a value in member az
 * @param _az New value for member az
 */
void MPU6050MSG::az(
        int16_t _az)
{
    m_az = _az;
}

/*!
 * @brief This function returns the value of member az
 * @return Value of member az
 */
int16_t MPU6050MSG::az() const
{
    return m_az;
}

/*!
 * @brief This function returns a reference to member az
 * @return Reference to member az
 */
int16_t& MPU6050MSG::az()
{
    return m_az;
}

/*!
 * @brief This function sets a value in member gx
 * @param _gx New value for member gx
 */
void MPU6050MSG::gx(
        int16_t _gx)
{
    m_gx = _gx;
}

/*!
 * @brief This function returns the value of member gx
 * @return Value of member gx
 */
int16_t MPU6050MSG::gx() const
{
    return m_gx;
}

/*!
 * @brief This function returns a reference to member gx
 * @return Reference to member gx
 */
int16_t& MPU6050MSG::gx()
{
    return m_gx;
}

/*!
 * @brief This function sets a value in member gy
 * @param _gy New value for member gy
 */
void MPU6050MSG::gy(
        int16_t _gy)
{
    m_gy = _gy;
}

/*!
 * @brief This function returns the value of member gy
 * @return Value of member gy
 */
int16_t MPU6050MSG::gy() const
{
    return m_gy;
}

/*!
 * @brief This function returns a reference to member gy
 * @return Reference to member gy
 */
int16_t& MPU6050MSG::gy()
{
    return m_gy;
}

/*!
 * @brief This function sets a value in member gz
 * @param _gz New value for member gz
 */
void MPU6050MSG::gz(
        int16_t _gz)
{
    m_gz = _gz;
}

/*!
 * @brief This function returns the value of member gz
 * @return Value of member gz
 */
int16_t MPU6050MSG::gz() const
{
    return m_gz;
}

/*!
 * @brief This function returns a reference to member gz
 * @return Reference to member gz
 */
int16_t& MPU6050MSG::gz()
{
    return m_gz;
}

/*!
 * @brief This function sets a value in member timestamp
 * @param _timestamp New value for member timestamp
 */
void MPU6050MSG::timestamp(
        int32_t _timestamp)
{
    m_timestamp = _timestamp;
}

/*!
 * @brief This function returns the value of member timestamp
 * @return Value of member timestamp
 */
int32_t MPU6050MSG::timestamp() const
{
    return m_timestamp;
}

/*!
 * @brief This function returns a reference to member timestamp
 * @return Reference to member timestamp
 */
int32_t& MPU6050MSG::timestamp()
{
    return m_timestamp;
}


size_t MPU6050MSG::getKeyMaxCdrSerializedSize(
        size_t current_alignment)
{
    size_t current_align = current_alignment;



    return current_align;
}

bool MPU6050MSG::isKeyDefined()
{
    return false;
}

void MPU6050MSG::serializeKey(
        eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
           
}
