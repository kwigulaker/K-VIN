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
 * @file FrameMSG.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _FAST_DDS_GENERATED_FRAMEMSG_H_
#define _FAST_DDS_GENERATED_FRAMEMSG_H_


#include <stdint.h>
#include <array>
#include <string>
#include <vector>
#include <map>
#include <bitset>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define eProsima_user_DllExport
#endif  // _WIN32

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(FrameMSG_SOURCE)
#define FrameMSG_DllAPI __declspec( dllexport )
#else
#define FrameMSG_DllAPI __declspec( dllimport )
#endif // FrameMSG_SOURCE
#else
#define FrameMSG_DllAPI
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define FrameMSG_DllAPI
#endif // _WIN32

namespace eprosima {
namespace fastcdr {
class Cdr;
} // namespace fastcdr
} // namespace eprosima


/*!
 * @brief This class represents the structure FrameMSG defined by the user in the IDL file.
 * @ingroup FRAMEMSG
 */
class FrameMSG
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport FrameMSG();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~FrameMSG();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object FrameMSG that will be copied.
     */
    eProsima_user_DllExport FrameMSG(
            const FrameMSG& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object FrameMSG that will be copied.
     */
    eProsima_user_DllExport FrameMSG(
            FrameMSG&& x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object FrameMSG that will be copied.
     */
    eProsima_user_DllExport FrameMSG& operator =(
            const FrameMSG& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object FrameMSG that will be copied.
     */
    eProsima_user_DllExport FrameMSG& operator =(
            FrameMSG&& x);

    /*!
     * @brief Comparison operator.
     * @param x FrameMSG object to compare.
     */
    eProsima_user_DllExport bool operator ==(
            const FrameMSG& x) const;

    /*!
     * @brief Comparison operator.
     * @param x FrameMSG object to compare.
     */
    eProsima_user_DllExport bool operator !=(
            const FrameMSG& x) const;

    /*!
     * @brief This function sets a value in member width
     * @param _width New value for member width
     */
    eProsima_user_DllExport void width(
            int32_t _width);

    /*!
     * @brief This function returns the value of member width
     * @return Value of member width
     */
    eProsima_user_DllExport int32_t width() const;

    /*!
     * @brief This function returns a reference to member width
     * @return Reference to member width
     */
    eProsima_user_DllExport int32_t& width();

    /*!
     * @brief This function sets a value in member height
     * @param _height New value for member height
     */
    eProsima_user_DllExport void height(
            int32_t _height);

    /*!
     * @brief This function returns the value of member height
     * @return Value of member height
     */
    eProsima_user_DllExport int32_t height() const;

    /*!
     * @brief This function returns a reference to member height
     * @return Reference to member height
     */
    eProsima_user_DllExport int32_t& height();

    /*!
     * @brief This function sets a value in member step
     * @param _step New value for member step
     */
    eProsima_user_DllExport void step(
            int32_t _step);

    /*!
     * @brief This function returns the value of member step
     * @return Value of member step
     */
    eProsima_user_DllExport int32_t step() const;

    /*!
     * @brief This function returns a reference to member step
     * @return Reference to member step
     */
    eProsima_user_DllExport int32_t& step();

    /*!
     * @brief This function sets a value in member timestamp
     * @param _timestamp New value for member timestamp
     */
    eProsima_user_DllExport void timestamp(
            int32_t _timestamp);

    /*!
     * @brief This function returns the value of member timestamp
     * @return Value of member timestamp
     */
    eProsima_user_DllExport int32_t timestamp() const;

    /*!
     * @brief This function returns a reference to member timestamp
     * @return Reference to member timestamp
     */
    eProsima_user_DllExport int32_t& timestamp();

    /*!
     * @brief This function copies the value in member encoding
     * @param _encoding New value to be copied in member encoding
     */
    eProsima_user_DllExport void encoding(
            const std::string& _encoding);

    /*!
     * @brief This function moves the value in member encoding
     * @param _encoding New value to be moved in member encoding
     */
    eProsima_user_DllExport void encoding(
            std::string&& _encoding);

    /*!
     * @brief This function returns a constant reference to member encoding
     * @return Constant reference to member encoding
     */
    eProsima_user_DllExport const std::string& encoding() const;

    /*!
     * @brief This function returns a reference to member encoding
     * @return Reference to member encoding
     */
    eProsima_user_DllExport std::string& encoding();
    /*!
     * @brief This function copies the value in member data
     * @param _data New value to be copied in member data
     */
    eProsima_user_DllExport void data(
            const std::vector<uint8_t>& _data);

    /*!
     * @brief This function moves the value in member data
     * @param _data New value to be moved in member data
     */
    eProsima_user_DllExport void data(
            std::vector<uint8_t>&& _data);

    /*!
     * @brief This function returns a constant reference to member data
     * @return Constant reference to member data
     */
    eProsima_user_DllExport const std::vector<uint8_t>& data() const;

    /*!
     * @brief This function returns a reference to member data
     * @return Reference to member data
     */
    eProsima_user_DllExport std::vector<uint8_t>& data();

    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getMaxCdrSerializedSize(
            size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    eProsima_user_DllExport static size_t getCdrSerializedSize(
            const FrameMSG& data,
            size_t current_alignment = 0);


    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serialize(
            eprosima::fastcdr::Cdr& cdr) const;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void deserialize(
            eprosima::fastcdr::Cdr& cdr);



    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(
            size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    eProsima_user_DllExport static bool isKeyDefined();

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serializeKey(
            eprosima::fastcdr::Cdr& cdr) const;

private:

    int32_t m_width;
    int32_t m_height;
    int32_t m_step;
    int32_t m_timestamp;
    std::string m_encoding;
    std::vector<uint8_t> m_data;
};

#endif // _FAST_DDS_GENERATED_FRAMEMSG_H_