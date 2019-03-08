//
// Generated file, do not edit! Created by nedtool 4.6 from satFrameRtn.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "satFrameRtn_m.h"

USING_NAMESPACE


// Another default rule (prevents compiler from choosing base class' doPacking())
template<typename T>
void doPacking(cCommBuffer *, T& t) {
    throw cRuntimeError("Parsim error: no doPacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}

template<typename T>
void doUnpacking(cCommBuffer *, T& t) {
    throw cRuntimeError("Parsim error: no doUnpacking() function for type %s or its base class (check .msg and _m.cc/h files!)",opp_typename(typeid(t)));
}




// Template rule for outputting std::vector<T> types
template<typename T, typename A>
inline std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec)
{
    out.put('{');
    for(typename std::vector<T,A>::const_iterator it = vec.begin(); it != vec.end(); ++it)
    {
        if (it != vec.begin()) {
            out.put(','); out.put(' ');
        }
        out << *it;
    }
    out.put('}');
    
    char buf[32];
    sprintf(buf, " (size=%u)", (unsigned int)vec.size());
    out.write(buf, strlen(buf));
    return out;
}

// Template rule which fires if a struct or class doesn't have operator<<
template<typename T>
inline std::ostream& operator<<(std::ostream& out,const T&) {return out;}

Register_Class(SatFrameRtn);

SatFrameRtn::SatFrameRtn(const char *name, int kind) : ::cPacket(name,kind)
{
    this->srcAddress_var = 0;
    this->randomSeed_var = 0;
    this->slotIndexWithinFrame_var = 0;
}

SatFrameRtn::SatFrameRtn(const SatFrameRtn& other) : ::cPacket(other)
{
    copy(other);
}

SatFrameRtn::~SatFrameRtn()
{
}

SatFrameRtn& SatFrameRtn::operator=(const SatFrameRtn& other)
{
    if (this==&other) return *this;
    ::cPacket::operator=(other);
    copy(other);
    return *this;
}

void SatFrameRtn::copy(const SatFrameRtn& other)
{
    this->srcAddress_var = other.srcAddress_var;
    this->randomSeed_var = other.randomSeed_var;
    this->slotIndexWithinFrame_var = other.slotIndexWithinFrame_var;
}

void SatFrameRtn::parsimPack(cCommBuffer *b)
{
    ::cPacket::parsimPack(b);
    doPacking(b,this->srcAddress_var);
    doPacking(b,this->randomSeed_var);
    doPacking(b,this->slotIndexWithinFrame_var);
}

void SatFrameRtn::parsimUnpack(cCommBuffer *b)
{
    ::cPacket::parsimUnpack(b);
    doUnpacking(b,this->srcAddress_var);
    doUnpacking(b,this->randomSeed_var);
    doUnpacking(b,this->slotIndexWithinFrame_var);
}

int SatFrameRtn::getSrcAddress() const
{
    return srcAddress_var;
}

void SatFrameRtn::setSrcAddress(int srcAddress)
{
    this->srcAddress_var = srcAddress;
}

int SatFrameRtn::getRandomSeed() const
{
    return randomSeed_var;
}

void SatFrameRtn::setRandomSeed(int randomSeed)
{
    this->randomSeed_var = randomSeed;
}

int SatFrameRtn::getSlotIndexWithinFrame() const
{
    return slotIndexWithinFrame_var;
}

void SatFrameRtn::setSlotIndexWithinFrame(int slotIndexWithinFrame)
{
    this->slotIndexWithinFrame_var = slotIndexWithinFrame;
}

class SatFrameRtnDescriptor : public cClassDescriptor
{
  public:
    SatFrameRtnDescriptor();
    virtual ~SatFrameRtnDescriptor();

    virtual bool doesSupport(cObject *obj) const;
    virtual const char *getProperty(const char *propertyname) const;
    virtual int getFieldCount(void *object) const;
    virtual const char *getFieldName(void *object, int field) const;
    virtual int findField(void *object, const char *fieldName) const;
    virtual unsigned int getFieldTypeFlags(void *object, int field) const;
    virtual const char *getFieldTypeString(void *object, int field) const;
    virtual const char *getFieldProperty(void *object, int field, const char *propertyname) const;
    virtual int getArraySize(void *object, int field) const;

    virtual std::string getFieldAsString(void *object, int field, int i) const;
    virtual bool setFieldAsString(void *object, int field, int i, const char *value) const;

    virtual const char *getFieldStructName(void *object, int field) const;
    virtual void *getFieldStructPointer(void *object, int field, int i) const;
};

Register_ClassDescriptor(SatFrameRtnDescriptor);

SatFrameRtnDescriptor::SatFrameRtnDescriptor() : cClassDescriptor("SatFrameRtn", "cPacket")
{
}

SatFrameRtnDescriptor::~SatFrameRtnDescriptor()
{
}

bool SatFrameRtnDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<SatFrameRtn *>(obj)!=NULL;
}

const char *SatFrameRtnDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int SatFrameRtnDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 3+basedesc->getFieldCount(object) : 3;
}

unsigned int SatFrameRtnDescriptor::getFieldTypeFlags(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeFlags(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
    };
    return (field>=0 && field<3) ? fieldTypeFlags[field] : 0;
}

const char *SatFrameRtnDescriptor::getFieldName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldNames[] = {
        "srcAddress",
        "randomSeed",
        "slotIndexWithinFrame",
    };
    return (field>=0 && field<3) ? fieldNames[field] : NULL;
}

int SatFrameRtnDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='s' && strcmp(fieldName, "srcAddress")==0) return base+0;
    if (fieldName[0]=='r' && strcmp(fieldName, "randomSeed")==0) return base+1;
    if (fieldName[0]=='s' && strcmp(fieldName, "slotIndexWithinFrame")==0) return base+2;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *SatFrameRtnDescriptor::getFieldTypeString(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeString(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldTypeStrings[] = {
        "int",
        "int",
        "int",
    };
    return (field>=0 && field<3) ? fieldTypeStrings[field] : NULL;
}

const char *SatFrameRtnDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldProperty(object, field, propertyname);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        default: return NULL;
    }
}

int SatFrameRtnDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    SatFrameRtn *pp = (SatFrameRtn *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string SatFrameRtnDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    SatFrameRtn *pp = (SatFrameRtn *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getSrcAddress());
        case 1: return long2string(pp->getRandomSeed());
        case 2: return long2string(pp->getSlotIndexWithinFrame());
        default: return "";
    }
}

bool SatFrameRtnDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    SatFrameRtn *pp = (SatFrameRtn *)object; (void)pp;
    switch (field) {
        case 0: pp->setSrcAddress(string2long(value)); return true;
        case 1: pp->setRandomSeed(string2long(value)); return true;
        case 2: pp->setSlotIndexWithinFrame(string2long(value)); return true;
        default: return false;
    }
}

const char *SatFrameRtnDescriptor::getFieldStructName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    switch (field) {
        default: return NULL;
    };
}

void *SatFrameRtnDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    SatFrameRtn *pp = (SatFrameRtn *)object; (void)pp;
    switch (field) {
        default: return NULL;
    }
}


