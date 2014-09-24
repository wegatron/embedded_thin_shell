/****************************************************************************
** Meta object code from reading C++ file 'RefineSoftbody.h'
**
** Created: Wed Aug 6 12:55:11 2014
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "RefineSoftbody.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'RefineSoftbody.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_RefineSoftbody[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      16,   15,   15,   15, 0x0a,
      25,   15,   15,   15, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_RefineSoftbody[] = {
    "RefineSoftbody\0\0browse()\0extcute()\0"
};

const QMetaObject RefineSoftbody::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_RefineSoftbody,
      qt_meta_data_RefineSoftbody, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &RefineSoftbody::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *RefineSoftbody::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *RefineSoftbody::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_RefineSoftbody))
        return static_cast<void*>(const_cast< RefineSoftbody*>(this));
    return QWidget::qt_metacast(_clname);
}

int RefineSoftbody::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: browse(); break;
        case 1: extcute(); break;
        default: ;
        }
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
