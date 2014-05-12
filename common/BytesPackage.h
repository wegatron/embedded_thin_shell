#ifndef __BYTES_PACKAGE_H__
#define __BYTES_PACKAGE_H__

#include "Config.hpp"
#include <vector>
#include <string>

PRJ_BEGIN

class BytesPackage {
  //BytesPackage(const BytesPackage & other);
  //BytesPackage & operator=(const BytesPackage & other);
public:
  BytesPackage()
  {
    reset();
  }
  virtual ~BytesPackage()
  {
  }

  inline const SizeType size() const
  {
    return _data.size();
  }
  inline const void * data() const
  {
    return (const void *)_data.data();
  }
  inline void * data()
  {
    return (void *)_data.data();
  }
  inline void reset()
  {
    _data.clear();
  }
  inline void resize(SizeType size)
  {
    _data.resize(size);
  }

  template <typename T>
  inline void add(const T & data)
  {
    const SizeType size = sizeof(T);
    const unsigned char * ptr = (const unsigned char *)(&data);
    for ( SizeType i=0; i < size; i++ )
    {
      _data.push_back(*ptr);
      ptr++;
    }
  }
  inline void addString(const std::string & s)
  {
    const size_t size = s.size();
    add(size);
    for ( size_t i=0; i < size; i++ )
    {
      add(s[i]);
    }
  }
  inline void addBlob(const void * data, const SizeType size)
  {
    const unsigned char * ptr = (const unsigned char *)(data);
    for ( SizeType i=0; i < size; i++ )
    {
      _data.push_back(*ptr);
      ptr++;
    }
  }
  template <typename T>
  inline void get(T & data)
  {
    const SizeType size = sizeof(T);
    unsigned char * ptr = (unsigned char *)(&data);
    ASSERT(_data.size()>=size);
    for ( SizeType i=0; i < size; i++ )
    {
      (*ptr) = _data[0];
      _data.erase(_data.begin());
      ptr++;
    }
  }
  inline void getString(std::string & s)
  {
    size_t size;
    get(size);
    s.resize(size);
    for ( size_t i=0; i < size; i++ )
    {
      get(s[i]);
    }
  }
  inline unsigned char * getBlob(const SizeType size)
  {
    unsigned char * blob = new unsigned char[size];
    for ( SizeType i=0; i < size; i++ )
    {
      blob[i] = _data[i];
    }
    _data.erase(_data.begin(), _data.begin()+size);
    return blob;
  }

private:
  std::vector<unsigned char> _data;

};

PRJ_END

#endif //__BYTES_PACKAGE_HPP__
