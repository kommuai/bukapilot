/****************************************************************************
** Meta object code from reading C++ file 'abstractstream.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "abstractstream.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'abstractstream.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_AbstractStream_t {
    QByteArrayData data[18];
    char stringdata0[214];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_AbstractStream_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_AbstractStream_t qt_meta_stringdata_AbstractStream = {
    {
QT_MOC_LITERAL(0, 0, 14), // "AbstractStream"
QT_MOC_LITERAL(1, 15, 6), // "paused"
QT_MOC_LITERAL(2, 22, 0), // ""
QT_MOC_LITERAL(3, 23, 6), // "resume"
QT_MOC_LITERAL(4, 30, 8), // "seekedTo"
QT_MOC_LITERAL(5, 39, 3), // "sec"
QT_MOC_LITERAL(6, 43, 13), // "streamStarted"
QT_MOC_LITERAL(7, 57, 12), // "eventsMerged"
QT_MOC_LITERAL(8, 70, 16), // "MessageEventsMap"
QT_MOC_LITERAL(9, 87, 10), // "events_map"
QT_MOC_LITERAL(10, 98, 12), // "msgsReceived"
QT_MOC_LITERAL(11, 111, 26), // "const std::set<MessageId>*"
QT_MOC_LITERAL(12, 138, 8), // "new_msgs"
QT_MOC_LITERAL(13, 147, 11), // "has_new_ids"
QT_MOC_LITERAL(14, 159, 14), // "sourcesUpdated"
QT_MOC_LITERAL(15, 174, 9), // "SourceSet"
QT_MOC_LITERAL(16, 184, 1), // "s"
QT_MOC_LITERAL(17, 186, 27) // "privateUpdateLastMsgsSignal"

    },
    "AbstractStream\0paused\0\0resume\0seekedTo\0"
    "sec\0streamStarted\0eventsMerged\0"
    "MessageEventsMap\0events_map\0msgsReceived\0"
    "const std::set<MessageId>*\0new_msgs\0"
    "has_new_ids\0sourcesUpdated\0SourceSet\0"
    "s\0privateUpdateLastMsgsSignal"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_AbstractStream[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       8,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   54,    2, 0x06 /* Public */,
       3,    0,   55,    2, 0x06 /* Public */,
       4,    1,   56,    2, 0x06 /* Public */,
       6,    0,   59,    2, 0x06 /* Public */,
       7,    1,   60,    2, 0x06 /* Public */,
      10,    2,   63,    2, 0x06 /* Public */,
      14,    1,   68,    2, 0x06 /* Public */,
      17,    0,   71,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double,    5,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 8,    9,
    QMetaType::Void, 0x80000000 | 11, QMetaType::Bool,   12,   13,
    QMetaType::Void, 0x80000000 | 15,   16,
    QMetaType::Void,

       0        // eod
};

void AbstractStream::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<AbstractStream *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->paused(); break;
        case 1: _t->resume(); break;
        case 2: _t->seekedTo((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 3: _t->streamStarted(); break;
        case 4: _t->eventsMerged((*reinterpret_cast< const MessageEventsMap(*)>(_a[1]))); break;
        case 5: _t->msgsReceived((*reinterpret_cast< const std::set<MessageId>*(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 6: _t->sourcesUpdated((*reinterpret_cast< const SourceSet(*)>(_a[1]))); break;
        case 7: _t->privateUpdateLastMsgsSignal(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (AbstractStream::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AbstractStream::paused)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (AbstractStream::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AbstractStream::resume)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (AbstractStream::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AbstractStream::seekedTo)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (AbstractStream::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AbstractStream::streamStarted)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (AbstractStream::*)(const MessageEventsMap & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AbstractStream::eventsMerged)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (AbstractStream::*)(const std::set<MessageId> * , bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AbstractStream::msgsReceived)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (AbstractStream::*)(const SourceSet & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AbstractStream::sourcesUpdated)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (AbstractStream::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AbstractStream::privateUpdateLastMsgsSignal)) {
                *result = 7;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject AbstractStream::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_AbstractStream.data,
    qt_meta_data_AbstractStream,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *AbstractStream::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *AbstractStream::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_AbstractStream.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int AbstractStream::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void AbstractStream::paused()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void AbstractStream::resume()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void AbstractStream::seekedTo(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void AbstractStream::streamStarted()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void AbstractStream::eventsMerged(const MessageEventsMap & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void AbstractStream::msgsReceived(const std::set<MessageId> * _t1, bool _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void AbstractStream::sourcesUpdated(const SourceSet & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void AbstractStream::privateUpdateLastMsgsSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 7, nullptr);
}
struct qt_meta_stringdata_DummyStream_t {
    QByteArrayData data[1];
    char stringdata0[12];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DummyStream_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DummyStream_t qt_meta_stringdata_DummyStream = {
    {
QT_MOC_LITERAL(0, 0, 11) // "DummyStream"

    },
    "DummyStream"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DummyStream[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void DummyStream::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    (void)_o;
    (void)_id;
    (void)_c;
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject DummyStream::staticMetaObject = { {
    QMetaObject::SuperData::link<AbstractStream::staticMetaObject>(),
    qt_meta_stringdata_DummyStream.data,
    qt_meta_data_DummyStream,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *DummyStream::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DummyStream::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DummyStream.stringdata0))
        return static_cast<void*>(this);
    return AbstractStream::qt_metacast(_clname);
}

int DummyStream::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = AbstractStream::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_StreamNotifier_t {
    QByteArrayData data[4];
    char stringdata0[45];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_StreamNotifier_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_StreamNotifier_t qt_meta_stringdata_StreamNotifier = {
    {
QT_MOC_LITERAL(0, 0, 14), // "StreamNotifier"
QT_MOC_LITERAL(1, 15, 13), // "streamStarted"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 14) // "changingStream"

    },
    "StreamNotifier\0streamStarted\0\0"
    "changingStream"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_StreamNotifier[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   24,    2, 0x06 /* Public */,
       3,    0,   25,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void StreamNotifier::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<StreamNotifier *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->streamStarted(); break;
        case 1: _t->changingStream(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (StreamNotifier::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&StreamNotifier::streamStarted)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (StreamNotifier::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&StreamNotifier::changingStream)) {
                *result = 1;
                return;
            }
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject StreamNotifier::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_StreamNotifier.data,
    qt_meta_data_StreamNotifier,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *StreamNotifier::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *StreamNotifier::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_StreamNotifier.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int StreamNotifier::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void StreamNotifier::streamStarted()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void StreamNotifier::changingStream()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
