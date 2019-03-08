//
// Generated file, do not edit! Created by nedtool 4.6 from satFrameFwd.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "satFrameFwd_m.h"

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

Register_Class(satFrameFwd);

satFrameFwd::satFrameFwd(const char *name, int kind) : ::cPacket(name,kind)
{
    this->destL2Address_var = 0;
    this->hasPayload_var = 0;
}

satFrameFwd::satFrameFwd(const satFrameFwd& other) : ::cPacket(other)
{
    copy(other);
}

satFrameFwd::~satFrameFwd()
{
}

satFrameFwd& satFrameFwd::operator=(const satFrameFwd& other)
{
    if (this==&other) return *this;
    ::cPacket::operator=(other);
    copy(other);
    return *this;
}

void satFrameFwd::copy(const satFrameFwd& other)
{
    this->destL2Address_var = other.destL2Address_var;
    this->hasPayload_var = other.hasPayload_var;
}

void satFrameFwd::parsimPack(cCommBuffer *b)
{
    ::cPacket::parsimPack(b);
    doPacking(b,this->destL2Address_var);
    doPacking(b,this->hasPayload_var);
}

void satFrameFwd::parsimUnpack(cCommBuffer *b)
{
    ::cPacket::parsimUnpack(b);
    doUnpacking(b,this->destL2Address_var);
    doUnpacking(b,this->hasPayload_var);
}

int satFrameFwd::getDestL2Address() const
{
    return destL2Address_var;
}

void satFrameFwd::setDestL2Address(int destL2Address)
{
    this->destL2Address_var = destL2Address;
}

bool satFrameFwd::getHasPayload() const
{
    return hasPayload_var;
}

void satFrameFwd::setHasPayload(bool hasPayload)
{
    this->hasPayload_var = hasPayload;
}

class satFrameFwdDescriptor : public cClassDescriptor
{
  public:
    satFrameFwdDescriptor();
    virtual ~satFrameFwdDescriptor();

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

Register_ClassDescriptor(satFrameFwdDescriptor);

satFrameFwdDescriptor::satFrameFwdDescriptor() : cClassDescriptor("satFrameFwd", "cPacket")
{
}

satFrameFwdDescriptor::~satFrameFwdDescriptor()
{
}

bool satFrameFwdDescriptor::doesSupport(cObject *obj) const
{
    return dynamic_cast<satFrameFwd *>(obj)!=NULL;
}

const char *satFrameFwdDescriptor::getProperty(const char *propertyname) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : NULL;
}

int satFrameFwdDescriptor::getFieldCount(void *object) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 2+basedesc->getFieldCount(object) : 2;
}

unsigned int satFrameFwdDescriptor::getFieldTypeFlags(void *object, int field) const
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
    };
    return (field>=0 && field<2) ? fieldTypeFlags[field] : 0;
}

const char *satFrameFwdDescriptor::getFieldName(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldName(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldNames[] = {
        "destL2Address",
        "hasPayload",
    };
    return (field>=0 && field<2) ? fieldNames[field] : NULL;
}

int satFrameFwdDescriptor::findField(void *object, const char *fieldName) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount(object) : 0;
    if (fieldName[0]=='d' && strcmp(fieldName, "destL2Address")==0) return base+0;
    if (fieldName[0]=='h' && strcmp(fieldName, "hasPayload")==0) return base+1;
    return basedesc ? basedesc->findField(object, fieldName) : -1;
}

const char *satFrameFwdDescriptor::getFieldTypeString(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldTypeString(object, field);
        field -= basedesc->getFieldCount(object);
    }
    static const char *fieldTypeStrings[] = {
        "int",
        "bool",
    };
    return (field>=0 && field<2) ? fieldTypeStrings[field] : NULL;
}

const char *satFrameFwdDescriptor::getFieldProperty(void *object, int field, const char *propertyname) const
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

int satFrameFwdDescriptor::getArraySize(void *object, int field) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getArraySize(object, field);
        field -= basedesc->getFieldCount(object);
    }
    satFrameFwd *pp = (satFrameFwd *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string satFrameFwdDescriptor::getFieldAsString(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldAsString(object,field,i);
        field -= basedesc->getFieldCount(object);
    }
    satFrameFwd *pp = (satFrameFwd *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getDestL2Address());
        case 1: return bool2string(pp->getHasPayload());
        default: return "";
    }
}

bool satFrameFwdDescriptor::setFieldAsString(void *object, int field, int i, const char *value) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->setFieldAsString(object,field,i,value);
        field -= basedesc->getFieldCount(object);
    }
    satFrameFwd *pp = (satFrameFwd *)object; (void)pp;
    switch (field) {
        case 0: pp->setDestL2Address(string2long(value)); return true;
        case 1: pp->setHasPayload(string2bool(value)); return true;
        default: return false;
    }
}

const char *satFrameFwdDescriptor::getFieldStructName(void *object, int field) const
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

void *satFrameFwdDescriptor::getFieldStructPointer(void *object, int field, int i) const
{
    cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount(object))
            return basedesc->getFieldStructPointer(object, field, i);
        field -= basedesc->getFieldCount(object);
    }
    satFrameFwd *pp = (satFrameFwd *)object; (void)pp;
    switch (field) {
        default: return NULL;
    }
}


