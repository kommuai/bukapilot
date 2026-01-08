/****************************************************************************
** Meta object code from reading C++ file 'binaryview.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "binaryview.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'binaryview.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_BinaryView_t {
    QByteArrayData data[15];
    char stringdata0[138];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_BinaryView_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_BinaryView_t qt_meta_stringdata_BinaryView = {
    {
QT_MOC_LITERAL(0, 0, 10), // "BinaryView"
QT_MOC_LITERAL(1, 11, 13), // "signalClicked"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 21), // "const cabana::Signal*"
QT_MOC_LITERAL(4, 48, 3), // "sig"
QT_MOC_LITERAL(5, 52, 13), // "signalHovered"
QT_MOC_LITERAL(6, 66, 10), // "editSignal"
QT_MOC_LITERAL(7, 77, 8), // "origin_s"
QT_MOC_LITERAL(8, 86, 15), // "cabana::Signal&"
QT_MOC_LITERAL(9, 102, 1), // "s"
QT_MOC_LITERAL(10, 104, 9), // "showChart"
QT_MOC_LITERAL(11, 114, 9), // "MessageId"
QT_MOC_LITERAL(12, 124, 2), // "id"
QT_MOC_LITERAL(13, 127, 4), // "show"
QT_MOC_LITERAL(14, 132, 5) // "merge"

    },
    "BinaryView\0signalClicked\0\0"
    "const cabana::Signal*\0sig\0signalHovered\0"
    "editSignal\0origin_s\0cabana::Signal&\0"
    "s\0showChart\0MessageId\0id\0show\0merge"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_BinaryView[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       5,    1,   37,    2, 0x06 /* Public */,
       6,    2,   40,    2, 0x06 /* Public */,
      10,    4,   45,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 8,    7,    9,
    QMetaType::Void, 0x80000000 | 11, 0x80000000 | 3, QMetaType::Bool, QMetaType::Bool,   12,    4,   13,   14,

       0        // eod
};

void BinaryView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<BinaryView *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->signalClicked((*reinterpret_cast< const cabana::Signal*(*)>(_a[1]))); break;
        case 1: _t->signalHovered((*reinterpret_cast< const cabana::Signal*(*)>(_a[1]))); break;
        case 2: _t->editSignal((*reinterpret_cast< const cabana::Signal*(*)>(_a[1])),(*reinterpret_cast< cabana::Signal(*)>(_a[2]))); break;
        case 3: _t->showChart((*reinterpret_cast< const MessageId(*)>(_a[1])),(*reinterpret_cast< const cabana::Signal*(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< MessageId >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (BinaryView::*)(const cabana::Signal * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&BinaryView::signalClicked)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (BinaryView::*)(const cabana::Signal * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&BinaryView::signalHovered)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (BinaryView::*)(const cabana::Signal * , cabana::Signal & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&BinaryView::editSignal)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (BinaryView::*)(const MessageId & , const cabana::Signal * , bool , bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&BinaryView::showChart)) {
                *result = 3;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject BinaryView::staticMetaObject = { {
    QMetaObject::SuperData::link<QTableView::staticMetaObject>(),
    qt_meta_stringdata_BinaryView.data,
    qt_meta_data_BinaryView,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *BinaryView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BinaryView::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_BinaryView.stringdata0))
        return static_cast<void*>(this);
    return QTableView::qt_metacast(_clname);
}

int BinaryView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTableView::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void BinaryView::signalClicked(const cabana::Signal * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void BinaryView::signalHovered(const cabana::Signal * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void BinaryView::editSignal(const cabana::Signal * _t1, cabana::Signal & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void BinaryView::showChart(const MessageId & _t1, const cabana::Signal * _t2, bool _t3, bool _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
