// Copyright (c) 2005-2014 Code Synthesis Tools CC
//
// This program was generated by CodeSynthesis XSD, an XML Schema to
// C++ data binding compiler.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
//
// In addition, as a special exception, Code Synthesis Tools CC gives
// permission to link this program with the Xerces-C++ library (or with
// modified versions of Xerces-C++ that use the same license as Xerces-C++),
// and distribute linked combinations including the two. You must obey
// the GNU General Public License version 2 in all respects for all of
// the code used other than Xerces-C++. If you modify this copy of the
// program, you may extend this exception to your version of the program,
// but you are not obligated to do so. If you do not wish to do so, delete
// this exception statement from your version.
//
// Furthermore, Code Synthesis Tools CC makes a special exception for
// the Free/Libre and Open Source Software (FLOSS) which is described
// in the accompanying FLOSSE file.
//

#ifndef NMSMODEL_HXX
#define NMSMODEL_HXX

#ifndef XSD_USE_CHAR
#define XSD_USE_CHAR
#endif

#ifndef XSD_CXX_TREE_USE_CHAR
#define XSD_CXX_TREE_USE_CHAR
#endif

// Begin prologue.
//
//
// End prologue.

#include <xsd/cxx/config.hxx>

#if (XSD_INT_VERSION != 4000000L)
#error XSD runtime version mismatch
#endif

#include <xsd/cxx/pre.hxx>

#include <xsd/cxx/xml/char-utf8.hxx>

#include <xsd/cxx/tree/exceptions.hxx>
#include <xsd/cxx/tree/elements.hxx>
#include <xsd/cxx/tree/types.hxx>

#include <xsd/cxx/xml/error-handler.hxx>

#include <xsd/cxx/xml/dom/auto-ptr.hxx>

#include <xsd/cxx/tree/parsing.hxx>
#include <xsd/cxx/tree/parsing/byte.hxx>
#include <xsd/cxx/tree/parsing/unsigned-byte.hxx>
#include <xsd/cxx/tree/parsing/short.hxx>
#include <xsd/cxx/tree/parsing/unsigned-short.hxx>
#include <xsd/cxx/tree/parsing/int.hxx>
#include <xsd/cxx/tree/parsing/unsigned-int.hxx>
#include <xsd/cxx/tree/parsing/long.hxx>
#include <xsd/cxx/tree/parsing/unsigned-long.hxx>
#include <xsd/cxx/tree/parsing/boolean.hxx>
#include <xsd/cxx/tree/parsing/float.hxx>
#include <xsd/cxx/tree/parsing/double.hxx>
#include <xsd/cxx/tree/parsing/decimal.hxx>

#include <xsd/cxx/xml/dom/serialization-header.hxx>
#include <xsd/cxx/tree/serialization.hxx>
#include <xsd/cxx/tree/serialization/byte.hxx>
#include <xsd/cxx/tree/serialization/unsigned-byte.hxx>
#include <xsd/cxx/tree/serialization/short.hxx>
#include <xsd/cxx/tree/serialization/unsigned-short.hxx>
#include <xsd/cxx/tree/serialization/int.hxx>
#include <xsd/cxx/tree/serialization/unsigned-int.hxx>
#include <xsd/cxx/tree/serialization/long.hxx>
#include <xsd/cxx/tree/serialization/unsigned-long.hxx>
#include <xsd/cxx/tree/serialization/boolean.hxx>
#include <xsd/cxx/tree/serialization/float.hxx>
#include <xsd/cxx/tree/serialization/double.hxx>
#include <xsd/cxx/tree/serialization/decimal.hxx>

namespace xml_schema
{
  // anyType and anySimpleType.
  //
  typedef ::xsd::cxx::tree::type type;
  typedef ::xsd::cxx::tree::simple_type< char, type > simple_type;
  typedef ::xsd::cxx::tree::type container;

  // 8-bit
  //
  typedef signed char byte;
  typedef unsigned char unsigned_byte;

  // 16-bit
  //
  typedef short short_;
  typedef unsigned short unsigned_short;

  // 32-bit
  //
  typedef int int_;
  typedef unsigned int unsigned_int;

  // 64-bit
  //
  typedef long long long_;
  typedef unsigned long long unsigned_long;

  // Supposed to be arbitrary-length integral types.
  //
  typedef long long integer;
  typedef long long non_positive_integer;
  typedef unsigned long long non_negative_integer;
  typedef unsigned long long positive_integer;
  typedef long long negative_integer;

  // Boolean.
  //
  typedef bool boolean;

  // Floating-point types.
  //
  typedef float float_;
  typedef double double_;
  typedef double decimal;

  // String types.
  //
  typedef ::xsd::cxx::tree::string< char, simple_type > string;
  typedef ::xsd::cxx::tree::normalized_string< char, string > normalized_string;
  typedef ::xsd::cxx::tree::token< char, normalized_string > token;
  typedef ::xsd::cxx::tree::name< char, token > name;
  typedef ::xsd::cxx::tree::nmtoken< char, token > nmtoken;
  typedef ::xsd::cxx::tree::nmtokens< char, simple_type, nmtoken > nmtokens;
  typedef ::xsd::cxx::tree::ncname< char, name > ncname;
  typedef ::xsd::cxx::tree::language< char, token > language;

  // ID/IDREF.
  //
  typedef ::xsd::cxx::tree::id< char, ncname > id;
  typedef ::xsd::cxx::tree::idref< char, ncname, type > idref;
  typedef ::xsd::cxx::tree::idrefs< char, simple_type, idref > idrefs;

  // URI.
  //
  typedef ::xsd::cxx::tree::uri< char, simple_type > uri;

  // Qualified name.
  //
  typedef ::xsd::cxx::tree::qname< char, simple_type, uri, ncname > qname;

  // Binary.
  //
  typedef ::xsd::cxx::tree::buffer< char > buffer;
  typedef ::xsd::cxx::tree::base64_binary< char, simple_type > base64_binary;
  typedef ::xsd::cxx::tree::hex_binary< char, simple_type > hex_binary;

  // Date/time.
  //
  typedef ::xsd::cxx::tree::time_zone time_zone;
  typedef ::xsd::cxx::tree::date< char, simple_type > date;
  typedef ::xsd::cxx::tree::date_time< char, simple_type > date_time;
  typedef ::xsd::cxx::tree::duration< char, simple_type > duration;
  typedef ::xsd::cxx::tree::gday< char, simple_type > gday;
  typedef ::xsd::cxx::tree::gmonth< char, simple_type > gmonth;
  typedef ::xsd::cxx::tree::gmonth_day< char, simple_type > gmonth_day;
  typedef ::xsd::cxx::tree::gyear< char, simple_type > gyear;
  typedef ::xsd::cxx::tree::gyear_month< char, simple_type > gyear_month;
  typedef ::xsd::cxx::tree::time< char, simple_type > time;

  // Entity.
  //
  typedef ::xsd::cxx::tree::entity< char, ncname > entity;
  typedef ::xsd::cxx::tree::entities< char, simple_type, entity > entities;

  typedef ::xsd::cxx::tree::content_order content_order;
  // Namespace information and list stream. Used in
  // serialization functions.
  //
  typedef ::xsd::cxx::xml::dom::namespace_info< char > namespace_info;
  typedef ::xsd::cxx::xml::dom::namespace_infomap< char > namespace_infomap;
  typedef ::xsd::cxx::tree::list_stream< char > list_stream;
  typedef ::xsd::cxx::tree::as_double< double_ > as_double;
  typedef ::xsd::cxx::tree::as_decimal< decimal > as_decimal;
  typedef ::xsd::cxx::tree::facet facet;

  // Flags and properties.
  //
  typedef ::xsd::cxx::tree::flags flags;
  typedef ::xsd::cxx::tree::properties< char > properties;

  // Parsing/serialization diagnostics.
  //
  typedef ::xsd::cxx::tree::severity severity;
  typedef ::xsd::cxx::tree::error< char > error;
  typedef ::xsd::cxx::tree::diagnostics< char > diagnostics;

  // Exceptions.
  //
  typedef ::xsd::cxx::tree::exception< char > exception;
  typedef ::xsd::cxx::tree::bounds< char > bounds;
  typedef ::xsd::cxx::tree::duplicate_id< char > duplicate_id;
  typedef ::xsd::cxx::tree::parsing< char > parsing;
  typedef ::xsd::cxx::tree::expected_element< char > expected_element;
  typedef ::xsd::cxx::tree::unexpected_element< char > unexpected_element;
  typedef ::xsd::cxx::tree::expected_attribute< char > expected_attribute;
  typedef ::xsd::cxx::tree::unexpected_enumerator< char > unexpected_enumerator;
  typedef ::xsd::cxx::tree::expected_text_content< char > expected_text_content;
  typedef ::xsd::cxx::tree::no_prefix_mapping< char > no_prefix_mapping;
  typedef ::xsd::cxx::tree::serialization< char > serialization;

  // Error handler callback interface.
  //
  typedef ::xsd::cxx::xml::error_handler< char > error_handler;

  // DOM interaction.
  //
  namespace dom
  {
    // Automatic pointer for DOMDocument.
    //
    using ::xsd::cxx::xml::dom::auto_ptr;

#ifndef XSD_CXX_TREE_TREE_NODE_KEY__XML_SCHEMA
#define XSD_CXX_TREE_TREE_NODE_KEY__XML_SCHEMA
    // DOM user data key for back pointers to tree nodes.
    //
    const XMLCh* const tree_node_key = ::xsd::cxx::tree::user_data_keys::node;
#endif
  }
}

// Forward declarations.
//
class MuscleType;
class MusclesType;
class MuscleSequenceType;
class ChannelType;
class ChannelsType;
class DoFType;
class DoFsType;
class PointsSequenceType;
class CurveType;
class MuscleDefaultType;
class NMSmodelType;

#include <memory>    // ::std::auto_ptr
#include <limits>    // std::numeric_limits
#include <algorithm> // std::binary_search

#include <xsd/cxx/xml/char-utf8.hxx>

#include <xsd/cxx/tree/exceptions.hxx>
#include <xsd/cxx/tree/elements.hxx>
#include <xsd/cxx/tree/containers.hxx>
#include <xsd/cxx/tree/list.hxx>

#include <xsd/cxx/xml/dom/parsing-header.hxx>

class MuscleType: public ::xml_schema::type
{
  public:
  // name
  //
  typedef ::xml_schema::string name_type;
  typedef ::xsd::cxx::tree::traits< name_type, char > name_traits;

  const name_type&
  name () const;

  name_type&
  name ();

  void
  name (const name_type& x);

  void
  name (::std::auto_ptr< name_type > p);

  // C1
  //
  typedef ::xml_schema::double_ C1_type;
  typedef ::xsd::cxx::tree::traits< C1_type, char, ::xsd::cxx::tree::schema_type::double_ > C1_traits;

  const C1_type&
  C1 () const;

  C1_type&
  C1 ();

  void
  C1 (const C1_type& x);

  // C2
  //
  typedef ::xml_schema::double_ C2_type;
  typedef ::xsd::cxx::tree::traits< C2_type, char, ::xsd::cxx::tree::schema_type::double_ > C2_traits;

  const C2_type&
  C2 () const;

  C2_type&
  C2 ();

  void
  C2 (const C2_type& x);

  // shapeFactor
  //
  typedef ::xml_schema::double_ shapeFactor_type;
  typedef ::xsd::cxx::tree::traits< shapeFactor_type, char, ::xsd::cxx::tree::schema_type::double_ > shapeFactor_traits;

  const shapeFactor_type&
  shapeFactor () const;

  shapeFactor_type&
  shapeFactor ();

  void
  shapeFactor (const shapeFactor_type& x);

  // optimalFiberLength
  //
  typedef ::xml_schema::double_ optimalFiberLength_type;
  typedef ::xsd::cxx::tree::traits< optimalFiberLength_type, char, ::xsd::cxx::tree::schema_type::double_ > optimalFiberLength_traits;

  const optimalFiberLength_type&
  optimalFiberLength () const;

  optimalFiberLength_type&
  optimalFiberLength ();

  void
  optimalFiberLength (const optimalFiberLength_type& x);

  // pennationAngle
  //
  typedef ::xml_schema::double_ pennationAngle_type;
  typedef ::xsd::cxx::tree::traits< pennationAngle_type, char, ::xsd::cxx::tree::schema_type::double_ > pennationAngle_traits;

  const pennationAngle_type&
  pennationAngle () const;

  pennationAngle_type&
  pennationAngle ();

  void
  pennationAngle (const pennationAngle_type& x);

  // tendonSlackLength
  //
  typedef ::xml_schema::double_ tendonSlackLength_type;
  typedef ::xsd::cxx::tree::traits< tendonSlackLength_type, char, ::xsd::cxx::tree::schema_type::double_ > tendonSlackLength_traits;

  const tendonSlackLength_type&
  tendonSlackLength () const;

  tendonSlackLength_type&
  tendonSlackLength ();

  void
  tendonSlackLength (const tendonSlackLength_type& x);

  // maxIsometricForce
  //
  typedef ::xml_schema::double_ maxIsometricForce_type;
  typedef ::xsd::cxx::tree::traits< maxIsometricForce_type, char, ::xsd::cxx::tree::schema_type::double_ > maxIsometricForce_traits;

  const maxIsometricForce_type&
  maxIsometricForce () const;

  maxIsometricForce_type&
  maxIsometricForce ();

  void
  maxIsometricForce (const maxIsometricForce_type& x);

  // strengthCoefficient
  //
  typedef ::xml_schema::double_ strengthCoefficient_type;
  typedef ::xsd::cxx::tree::traits< strengthCoefficient_type, char, ::xsd::cxx::tree::schema_type::double_ > strengthCoefficient_traits;

  const strengthCoefficient_type&
  strengthCoefficient () const;

  strengthCoefficient_type&
  strengthCoefficient ();

  void
  strengthCoefficient (const strengthCoefficient_type& x);

  // Constructors.
  //
  MuscleType (const name_type&,
              const C1_type&,
              const C2_type&,
              const shapeFactor_type&,
              const optimalFiberLength_type&,
              const pennationAngle_type&,
              const tendonSlackLength_type&,
              const maxIsometricForce_type&,
              const strengthCoefficient_type&);

  MuscleType (const ::xercesc::DOMElement& e,
              ::xml_schema::flags f = 0,
              ::xml_schema::container* c = 0);

  MuscleType (const MuscleType& x,
              ::xml_schema::flags f = 0,
              ::xml_schema::container* c = 0);

  virtual MuscleType*
  _clone (::xml_schema::flags f = 0,
          ::xml_schema::container* c = 0) const;

  MuscleType&
  operator= (const MuscleType& x);

  virtual 
  ~MuscleType ();

  // Implementation.
  //
  protected:
  void
  parse (::xsd::cxx::xml::dom::parser< char >&,
         ::xml_schema::flags);

  protected:
  ::xsd::cxx::tree::one< name_type > name_;
  ::xsd::cxx::tree::one< C1_type > C1_;
  ::xsd::cxx::tree::one< C2_type > C2_;
  ::xsd::cxx::tree::one< shapeFactor_type > shapeFactor_;
  ::xsd::cxx::tree::one< optimalFiberLength_type > optimalFiberLength_;
  ::xsd::cxx::tree::one< pennationAngle_type > pennationAngle_;
  ::xsd::cxx::tree::one< tendonSlackLength_type > tendonSlackLength_;
  ::xsd::cxx::tree::one< maxIsometricForce_type > maxIsometricForce_;
  ::xsd::cxx::tree::one< strengthCoefficient_type > strengthCoefficient_;
};

class MusclesType: public ::xml_schema::type
{
  public:
  // muscle
  //
  typedef ::MuscleType muscle_type;
  typedef ::xsd::cxx::tree::sequence< muscle_type > muscle_sequence;
  typedef muscle_sequence::iterator muscle_iterator;
  typedef muscle_sequence::const_iterator muscle_const_iterator;
  typedef ::xsd::cxx::tree::traits< muscle_type, char > muscle_traits;

  const muscle_sequence&
  muscle () const;

  muscle_sequence&
  muscle ();

  void
  muscle (const muscle_sequence& s);

  // Constructors.
  //
  MusclesType ();

  MusclesType (const ::xercesc::DOMElement& e,
               ::xml_schema::flags f = 0,
               ::xml_schema::container* c = 0);

  MusclesType (const MusclesType& x,
               ::xml_schema::flags f = 0,
               ::xml_schema::container* c = 0);

  virtual MusclesType*
  _clone (::xml_schema::flags f = 0,
          ::xml_schema::container* c = 0) const;

  MusclesType&
  operator= (const MusclesType& x);

  virtual 
  ~MusclesType ();

  // Implementation.
  //
  protected:
  void
  parse (::xsd::cxx::xml::dom::parser< char >&,
         ::xml_schema::flags);

  protected:
  muscle_sequence muscle_;
};

class MuscleSequenceType: public ::xml_schema::simple_type,
  public ::xsd::cxx::tree::list< ::xml_schema::string, char >
{
  public:
  MuscleSequenceType ();

  MuscleSequenceType (size_type n, const ::xml_schema::string& x);

  template < typename I >
  MuscleSequenceType (const I& begin, const I& end)
  : ::xsd::cxx::tree::list< ::xml_schema::string, char > (begin, end, this)
  {
  }

  MuscleSequenceType (const ::xercesc::DOMElement& e,
                      ::xml_schema::flags f = 0,
                      ::xml_schema::container* c = 0);

  MuscleSequenceType (const ::xercesc::DOMAttr& a,
                      ::xml_schema::flags f = 0,
                      ::xml_schema::container* c = 0);

  MuscleSequenceType (const ::std::string& s,
                      const ::xercesc::DOMElement* e,
                      ::xml_schema::flags f = 0,
                      ::xml_schema::container* c = 0);

  MuscleSequenceType (const MuscleSequenceType& x,
                      ::xml_schema::flags f = 0,
                      ::xml_schema::container* c = 0);

  virtual MuscleSequenceType*
  _clone (::xml_schema::flags f = 0,
          ::xml_schema::container* c = 0) const;

  virtual 
  ~MuscleSequenceType ();
};

class ChannelType: public ::xml_schema::type
{
  public:
  // name
  //
  typedef ::xml_schema::string name_type;
  typedef ::xsd::cxx::tree::traits< name_type, char > name_traits;

  const name_type&
  name () const;

  name_type&
  name ();

  void
  name (const name_type& x);

  void
  name (::std::auto_ptr< name_type > p);

  // muscleSequence
  //
  typedef ::MuscleSequenceType muscleSequence_type;
  typedef ::xsd::cxx::tree::traits< muscleSequence_type, char > muscleSequence_traits;

  const muscleSequence_type&
  muscleSequence () const;

  muscleSequence_type&
  muscleSequence ();

  void
  muscleSequence (const muscleSequence_type& x);

  void
  muscleSequence (::std::auto_ptr< muscleSequence_type > p);

  // Constructors.
  //
  ChannelType (const name_type&,
               const muscleSequence_type&);

  ChannelType (const ::xercesc::DOMElement& e,
               ::xml_schema::flags f = 0,
               ::xml_schema::container* c = 0);

  ChannelType (const ChannelType& x,
               ::xml_schema::flags f = 0,
               ::xml_schema::container* c = 0);

  virtual ChannelType*
  _clone (::xml_schema::flags f = 0,
          ::xml_schema::container* c = 0) const;

  ChannelType&
  operator= (const ChannelType& x);

  virtual 
  ~ChannelType ();

  // Implementation.
  //
  protected:
  void
  parse (::xsd::cxx::xml::dom::parser< char >&,
         ::xml_schema::flags);

  protected:
  ::xsd::cxx::tree::one< name_type > name_;
  ::xsd::cxx::tree::one< muscleSequence_type > muscleSequence_;
};

class ChannelsType: public ::xml_schema::type
{
  public:
  // Channel
  //
  typedef ::ChannelType Channel_type;
  typedef ::xsd::cxx::tree::sequence< Channel_type > Channel_sequence;
  typedef Channel_sequence::iterator Channel_iterator;
  typedef Channel_sequence::const_iterator Channel_const_iterator;
  typedef ::xsd::cxx::tree::traits< Channel_type, char > Channel_traits;

  const Channel_sequence&
  Channel () const;

  Channel_sequence&
  Channel ();

  void
  Channel (const Channel_sequence& s);

  // Constructors.
  //
  ChannelsType ();

  ChannelsType (const ::xercesc::DOMElement& e,
                ::xml_schema::flags f = 0,
                ::xml_schema::container* c = 0);

  ChannelsType (const ChannelsType& x,
                ::xml_schema::flags f = 0,
                ::xml_schema::container* c = 0);

  virtual ChannelsType*
  _clone (::xml_schema::flags f = 0,
          ::xml_schema::container* c = 0) const;

  ChannelsType&
  operator= (const ChannelsType& x);

  virtual 
  ~ChannelsType ();

  // Implementation.
  //
  protected:
  void
  parse (::xsd::cxx::xml::dom::parser< char >&,
         ::xml_schema::flags);

  protected:
  Channel_sequence Channel_;
};

class DoFType: public ::xml_schema::type
{
  public:
  // name
  //
  typedef ::xml_schema::string name_type;
  typedef ::xsd::cxx::tree::traits< name_type, char > name_traits;

  const name_type&
  name () const;

  name_type&
  name ();

  void
  name (const name_type& x);

  void
  name (::std::auto_ptr< name_type > p);

  // muscleSequence
  //
  typedef ::MuscleSequenceType muscleSequence_type;
  typedef ::xsd::cxx::tree::traits< muscleSequence_type, char > muscleSequence_traits;

  const muscleSequence_type&
  muscleSequence () const;

  muscleSequence_type&
  muscleSequence ();

  void
  muscleSequence (const muscleSequence_type& x);

  void
  muscleSequence (::std::auto_ptr< muscleSequence_type > p);

  // Constructors.
  //
  DoFType (const name_type&,
           const muscleSequence_type&);

  DoFType (const ::xercesc::DOMElement& e,
           ::xml_schema::flags f = 0,
           ::xml_schema::container* c = 0);

  DoFType (const DoFType& x,
           ::xml_schema::flags f = 0,
           ::xml_schema::container* c = 0);

  virtual DoFType*
  _clone (::xml_schema::flags f = 0,
          ::xml_schema::container* c = 0) const;

  DoFType&
  operator= (const DoFType& x);

  virtual 
  ~DoFType ();

  // Implementation.
  //
  protected:
  void
  parse (::xsd::cxx::xml::dom::parser< char >&,
         ::xml_schema::flags);

  protected:
  ::xsd::cxx::tree::one< name_type > name_;
  ::xsd::cxx::tree::one< muscleSequence_type > muscleSequence_;
};

class DoFsType: public ::xml_schema::type
{
  public:
  // DoF
  //
  typedef ::DoFType DoF_type;
  typedef ::xsd::cxx::tree::sequence< DoF_type > DoF_sequence;
  typedef DoF_sequence::iterator DoF_iterator;
  typedef DoF_sequence::const_iterator DoF_const_iterator;
  typedef ::xsd::cxx::tree::traits< DoF_type, char > DoF_traits;

  const DoF_sequence&
  DoF () const;

  DoF_sequence&
  DoF ();

  void
  DoF (const DoF_sequence& s);

  // Constructors.
  //
  DoFsType ();

  DoFsType (const ::xercesc::DOMElement& e,
            ::xml_schema::flags f = 0,
            ::xml_schema::container* c = 0);

  DoFsType (const DoFsType& x,
            ::xml_schema::flags f = 0,
            ::xml_schema::container* c = 0);

  virtual DoFsType*
  _clone (::xml_schema::flags f = 0,
          ::xml_schema::container* c = 0) const;

  DoFsType&
  operator= (const DoFsType& x);

  virtual 
  ~DoFsType ();

  // Implementation.
  //
  protected:
  void
  parse (::xsd::cxx::xml::dom::parser< char >&,
         ::xml_schema::flags);

  protected:
  DoF_sequence DoF_;
};

class PointsSequenceType: public ::xml_schema::simple_type,
  public ::xsd::cxx::tree::list< ::xml_schema::double_, char, ::xsd::cxx::tree::schema_type::double_ >
{
  public:
  PointsSequenceType ();

  PointsSequenceType (size_type n, const ::xml_schema::double_& x);

  template < typename I >
  PointsSequenceType (const I& begin, const I& end)
  : ::xsd::cxx::tree::list< ::xml_schema::double_, char, ::xsd::cxx::tree::schema_type::double_ > (begin, end, this)
  {
  }

  PointsSequenceType (const ::xercesc::DOMElement& e,
                      ::xml_schema::flags f = 0,
                      ::xml_schema::container* c = 0);

  PointsSequenceType (const ::xercesc::DOMAttr& a,
                      ::xml_schema::flags f = 0,
                      ::xml_schema::container* c = 0);

  PointsSequenceType (const ::std::string& s,
                      const ::xercesc::DOMElement* e,
                      ::xml_schema::flags f = 0,
                      ::xml_schema::container* c = 0);

  PointsSequenceType (const PointsSequenceType& x,
                      ::xml_schema::flags f = 0,
                      ::xml_schema::container* c = 0);

  virtual PointsSequenceType*
  _clone (::xml_schema::flags f = 0,
          ::xml_schema::container* c = 0) const;

  virtual 
  ~PointsSequenceType ();
};

class CurveType: public ::xml_schema::type
{
  public:
  // name
  //
  typedef ::xml_schema::string name_type;
  typedef ::xsd::cxx::tree::traits< name_type, char > name_traits;

  const name_type&
  name () const;

  name_type&
  name ();

  void
  name (const name_type& x);

  void
  name (::std::auto_ptr< name_type > p);

  // xPoints
  //
  typedef ::PointsSequenceType xPoints_type;
  typedef ::xsd::cxx::tree::traits< xPoints_type, char > xPoints_traits;

  const xPoints_type&
  xPoints () const;

  xPoints_type&
  xPoints ();

  void
  xPoints (const xPoints_type& x);

  void
  xPoints (::std::auto_ptr< xPoints_type > p);

  // yPoints
  //
  typedef ::PointsSequenceType yPoints_type;
  typedef ::xsd::cxx::tree::traits< yPoints_type, char > yPoints_traits;

  const yPoints_type&
  yPoints () const;

  yPoints_type&
  yPoints ();

  void
  yPoints (const yPoints_type& x);

  void
  yPoints (::std::auto_ptr< yPoints_type > p);

  // Constructors.
  //
  CurveType (const name_type&,
             const xPoints_type&,
             const yPoints_type&);

  CurveType (const ::xercesc::DOMElement& e,
             ::xml_schema::flags f = 0,
             ::xml_schema::container* c = 0);

  CurveType (const CurveType& x,
             ::xml_schema::flags f = 0,
             ::xml_schema::container* c = 0);

  virtual CurveType*
  _clone (::xml_schema::flags f = 0,
          ::xml_schema::container* c = 0) const;

  CurveType&
  operator= (const CurveType& x);

  virtual 
  ~CurveType ();

  // Implementation.
  //
  protected:
  void
  parse (::xsd::cxx::xml::dom::parser< char >&,
         ::xml_schema::flags);

  protected:
  ::xsd::cxx::tree::one< name_type > name_;
  ::xsd::cxx::tree::one< xPoints_type > xPoints_;
  ::xsd::cxx::tree::one< yPoints_type > yPoints_;
};

class MuscleDefaultType: public ::xml_schema::type
{
  public:
  // percentageChange
  //
  typedef ::xml_schema::double_ percentageChange_type;
  typedef ::xsd::cxx::tree::traits< percentageChange_type, char, ::xsd::cxx::tree::schema_type::double_ > percentageChange_traits;

  const percentageChange_type&
  percentageChange () const;

  percentageChange_type&
  percentageChange ();

  void
  percentageChange (const percentageChange_type& x);

  // damping
  //
  typedef ::xml_schema::double_ damping_type;
  typedef ::xsd::cxx::tree::traits< damping_type, char, ::xsd::cxx::tree::schema_type::double_ > damping_traits;

  const damping_type&
  damping () const;

  damping_type&
  damping ();

  void
  damping (const damping_type& x);

  // Curve
  //
  typedef ::CurveType Curve_type;
  typedef ::xsd::cxx::tree::sequence< Curve_type > Curve_sequence;
  typedef Curve_sequence::iterator Curve_iterator;
  typedef Curve_sequence::const_iterator Curve_const_iterator;
  typedef ::xsd::cxx::tree::traits< Curve_type, char > Curve_traits;

  const Curve_sequence&
  Curve () const;

  Curve_sequence&
  Curve ();

  void
  Curve (const Curve_sequence& s);

  // Constructors.
  //
  MuscleDefaultType (const percentageChange_type&,
                     const damping_type&);

  MuscleDefaultType (const ::xercesc::DOMElement& e,
                     ::xml_schema::flags f = 0,
                     ::xml_schema::container* c = 0);

  MuscleDefaultType (const MuscleDefaultType& x,
                     ::xml_schema::flags f = 0,
                     ::xml_schema::container* c = 0);

  virtual MuscleDefaultType*
  _clone (::xml_schema::flags f = 0,
          ::xml_schema::container* c = 0) const;

  MuscleDefaultType&
  operator= (const MuscleDefaultType& x);

  virtual 
  ~MuscleDefaultType ();

  // Implementation.
  //
  protected:
  void
  parse (::xsd::cxx::xml::dom::parser< char >&,
         ::xml_schema::flags);

  protected:
  ::xsd::cxx::tree::one< percentageChange_type > percentageChange_;
  ::xsd::cxx::tree::one< damping_type > damping_;
  Curve_sequence Curve_;
};

class NMSmodelType: public ::xml_schema::type
{
  public:
  // muscleDefault
  //
  typedef ::MuscleDefaultType muscleDefault_type;
  typedef ::xsd::cxx::tree::traits< muscleDefault_type, char > muscleDefault_traits;

  const muscleDefault_type&
  muscleDefault () const;

  muscleDefault_type&
  muscleDefault ();

  void
  muscleDefault (const muscleDefault_type& x);

  void
  muscleDefault (::std::auto_ptr< muscleDefault_type > p);

  // muscles
  //
  typedef ::MusclesType muscles_type;
  typedef ::xsd::cxx::tree::traits< muscles_type, char > muscles_traits;

  const muscles_type&
  muscles () const;

  muscles_type&
  muscles ();

  void
  muscles (const muscles_type& x);

  void
  muscles (::std::auto_ptr< muscles_type > p);

  // DoFs
  //
  typedef ::DoFsType DoFs_type;
  typedef ::xsd::cxx::tree::traits< DoFs_type, char > DoFs_traits;

  const DoFs_type&
  DoFs () const;

  DoFs_type&
  DoFs ();

  void
  DoFs (const DoFs_type& x);

  void
  DoFs (::std::auto_ptr< DoFs_type > p);

  // Channels
  //
  typedef ::ChannelsType Channels_type;
  typedef ::xsd::cxx::tree::traits< Channels_type, char > Channels_traits;

  const Channels_type&
  Channels () const;

  Channels_type&
  Channels ();

  void
  Channels (const Channels_type& x);

  void
  Channels (::std::auto_ptr< Channels_type > p);

  // Constructors.
  //
  NMSmodelType (const muscleDefault_type&,
                const muscles_type&,
                const DoFs_type&,
                const Channels_type&);

  NMSmodelType (::std::auto_ptr< muscleDefault_type >,
                ::std::auto_ptr< muscles_type >,
                ::std::auto_ptr< DoFs_type >,
                ::std::auto_ptr< Channels_type >);

  NMSmodelType (const ::xercesc::DOMElement& e,
                ::xml_schema::flags f = 0,
                ::xml_schema::container* c = 0);

  NMSmodelType (const NMSmodelType& x,
                ::xml_schema::flags f = 0,
                ::xml_schema::container* c = 0);

  virtual NMSmodelType*
  _clone (::xml_schema::flags f = 0,
          ::xml_schema::container* c = 0) const;

  NMSmodelType&
  operator= (const NMSmodelType& x);

  virtual 
  ~NMSmodelType ();

  // Implementation.
  //
  protected:
  void
  parse (::xsd::cxx::xml::dom::parser< char >&,
         ::xml_schema::flags);

  protected:
  ::xsd::cxx::tree::one< muscleDefault_type > muscleDefault_;
  ::xsd::cxx::tree::one< muscles_type > muscles_;
  ::xsd::cxx::tree::one< DoFs_type > DoFs_;
  ::xsd::cxx::tree::one< Channels_type > Channels_;
};

#include <iosfwd>

#include <xercesc/sax/InputSource.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMErrorHandler.hpp>

// Parse a URI or a local file.
//

::std::auto_ptr< ::NMSmodelType >
subject (const ::std::string& uri,
         ::xml_schema::flags f = 0,
         const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::NMSmodelType >
subject (const ::std::string& uri,
         ::xml_schema::error_handler& eh,
         ::xml_schema::flags f = 0,
         const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::NMSmodelType >
subject (const ::std::string& uri,
         ::xercesc::DOMErrorHandler& eh,
         ::xml_schema::flags f = 0,
         const ::xml_schema::properties& p = ::xml_schema::properties ());

// Parse std::istream.
//

::std::auto_ptr< ::NMSmodelType >
subject (::std::istream& is,
         ::xml_schema::flags f = 0,
         const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::NMSmodelType >
subject (::std::istream& is,
         ::xml_schema::error_handler& eh,
         ::xml_schema::flags f = 0,
         const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::NMSmodelType >
subject (::std::istream& is,
         ::xercesc::DOMErrorHandler& eh,
         ::xml_schema::flags f = 0,
         const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::NMSmodelType >
subject (::std::istream& is,
         const ::std::string& id,
         ::xml_schema::flags f = 0,
         const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::NMSmodelType >
subject (::std::istream& is,
         const ::std::string& id,
         ::xml_schema::error_handler& eh,
         ::xml_schema::flags f = 0,
         const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::NMSmodelType >
subject (::std::istream& is,
         const ::std::string& id,
         ::xercesc::DOMErrorHandler& eh,
         ::xml_schema::flags f = 0,
         const ::xml_schema::properties& p = ::xml_schema::properties ());

// Parse xercesc::InputSource.
//

::std::auto_ptr< ::NMSmodelType >
subject (::xercesc::InputSource& is,
         ::xml_schema::flags f = 0,
         const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::NMSmodelType >
subject (::xercesc::InputSource& is,
         ::xml_schema::error_handler& eh,
         ::xml_schema::flags f = 0,
         const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::NMSmodelType >
subject (::xercesc::InputSource& is,
         ::xercesc::DOMErrorHandler& eh,
         ::xml_schema::flags f = 0,
         const ::xml_schema::properties& p = ::xml_schema::properties ());

// Parse xercesc::DOMDocument.
//

::std::auto_ptr< ::NMSmodelType >
subject (const ::xercesc::DOMDocument& d,
         ::xml_schema::flags f = 0,
         const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::NMSmodelType >
subject (::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d,
         ::xml_schema::flags f = 0,
         const ::xml_schema::properties& p = ::xml_schema::properties ());

#include <iosfwd>

#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMErrorHandler.hpp>
#include <xercesc/framework/XMLFormatter.hpp>

#include <xsd/cxx/xml/dom/auto-ptr.hxx>

void
operator<< (::xercesc::DOMElement&, const MuscleType&);

void
operator<< (::xercesc::DOMElement&, const MusclesType&);

void
operator<< (::xercesc::DOMElement&, const MuscleSequenceType&);

void
operator<< (::xercesc::DOMAttr&, const MuscleSequenceType&);

void
operator<< (::xml_schema::list_stream&,
            const MuscleSequenceType&);

void
operator<< (::xercesc::DOMElement&, const ChannelType&);

void
operator<< (::xercesc::DOMElement&, const ChannelsType&);

void
operator<< (::xercesc::DOMElement&, const DoFType&);

void
operator<< (::xercesc::DOMElement&, const DoFsType&);

void
operator<< (::xercesc::DOMElement&, const PointsSequenceType&);

void
operator<< (::xercesc::DOMAttr&, const PointsSequenceType&);

void
operator<< (::xml_schema::list_stream&,
            const PointsSequenceType&);

void
operator<< (::xercesc::DOMElement&, const CurveType&);

void
operator<< (::xercesc::DOMElement&, const MuscleDefaultType&);

void
operator<< (::xercesc::DOMElement&, const NMSmodelType&);

// Serialize to std::ostream.
//

void
subject (::std::ostream& os,
         const ::NMSmodelType& x, 
         const ::xml_schema::namespace_infomap& m = ::xml_schema::namespace_infomap (),
         const ::std::string& e = "UTF-8",
         ::xml_schema::flags f = 0);

void
subject (::std::ostream& os,
         const ::NMSmodelType& x, 
         ::xml_schema::error_handler& eh,
         const ::xml_schema::namespace_infomap& m = ::xml_schema::namespace_infomap (),
         const ::std::string& e = "UTF-8",
         ::xml_schema::flags f = 0);

void
subject (::std::ostream& os,
         const ::NMSmodelType& x, 
         ::xercesc::DOMErrorHandler& eh,
         const ::xml_schema::namespace_infomap& m = ::xml_schema::namespace_infomap (),
         const ::std::string& e = "UTF-8",
         ::xml_schema::flags f = 0);

// Serialize to xercesc::XMLFormatTarget.
//

void
subject (::xercesc::XMLFormatTarget& ft,
         const ::NMSmodelType& x, 
         const ::xml_schema::namespace_infomap& m = ::xml_schema::namespace_infomap (),
         const ::std::string& e = "UTF-8",
         ::xml_schema::flags f = 0);

void
subject (::xercesc::XMLFormatTarget& ft,
         const ::NMSmodelType& x, 
         ::xml_schema::error_handler& eh,
         const ::xml_schema::namespace_infomap& m = ::xml_schema::namespace_infomap (),
         const ::std::string& e = "UTF-8",
         ::xml_schema::flags f = 0);

void
subject (::xercesc::XMLFormatTarget& ft,
         const ::NMSmodelType& x, 
         ::xercesc::DOMErrorHandler& eh,
         const ::xml_schema::namespace_infomap& m = ::xml_schema::namespace_infomap (),
         const ::std::string& e = "UTF-8",
         ::xml_schema::flags f = 0);

// Serialize to an existing xercesc::DOMDocument.
//

void
subject (::xercesc::DOMDocument& d,
         const ::NMSmodelType& x,
         ::xml_schema::flags f = 0);

// Serialize to a new xercesc::DOMDocument.
//

::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument >
subject (const ::NMSmodelType& x, 
         const ::xml_schema::namespace_infomap& m = ::xml_schema::namespace_infomap (),
         ::xml_schema::flags f = 0);

#include <xsd/cxx/post.hxx>

// Begin epilogue.
//
//
// End epilogue.

#endif // NMSMODEL_HXX