/****************************************************************************
** Meta object code from reading C++ file 'replay.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "replay.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'replay.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Replay_t {
    QByteArrayData data[12];
    char stringdata0[128];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Replay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Replay_t qt_meta_stringdata_Replay = {
    {
QT_MOC_LITERAL(0, 0, 6), // "Replay"
QT_MOC_LITERAL(1, 7, 13), // "streamStarted"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 14), // "segmentsMerged"
QT_MOC_LITERAL(4, 37, 8), // "seekedTo"
QT_MOC_LITERAL(5, 46, 3), // "sec"
QT_MOC_LITERAL(6, 50, 10), // "qLogLoaded"
QT_MOC_LITERAL(7, 61, 6), // "segnum"
QT_MOC_LITERAL(8, 68, 26), // "std::shared_ptr<LogReader>"
QT_MOC_LITERAL(9, 95, 4), // "qlog"
QT_MOC_LITERAL(10, 100, 19), // "segmentLoadFinished"
QT_MOC_LITERAL(11, 120, 7) // "success"

    },
    "Replay\0streamStarted\0\0segmentsMerged\0"
    "seekedTo\0sec\0qLogLoaded\0segnum\0"
    "std::shared_ptr<LogReader>\0qlog\0"
    "segmentLoadFinished\0success"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Replay[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x06 /* Public */,
       3,    0,   40,    2, 0x06 /* Public */,
       4,    1,   41,    2, 0x06 /* Public */,
       6,    2,   44,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      10,    1,   49,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double,    5,
    QMetaType::Void, QMetaType::Int, 0x80000000 | 8,    7,    9,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,   11,

       0        // eod
};

void Replay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Replay *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->streamStarted(); break;
        case 1: _t->segmentsMerged(); break;
        case 2: _t->seekedTo((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 3: _t->qLogLoaded((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< std::shared_ptr<LogReader>(*)>(_a[2]))); break;
        case 4: _t->segmentLoadFinished((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< std::shared_ptr<LogReader> >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Replay::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Replay::streamStarted)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (Replay::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Replay::segmentsMerged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (Replay::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Replay::seekedTo)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (Replay::*)(int , std::shared_ptr<LogReader> );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Replay::qLogLoaded)) {
                *result = 3;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Replay::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_Replay.data,
    qt_meta_data_Replay,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Replay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Replay::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Replay.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int Replay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void Replay::streamStarted()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void Replay::segmentsMerged()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void Replay::seekedTo(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void Replay::qLogLoaded(int _t1, std::shared_ptr<LogReader> _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
