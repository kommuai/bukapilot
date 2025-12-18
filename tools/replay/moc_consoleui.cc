/****************************************************************************
** Meta object code from reading C++ file 'consoleui.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "consoleui.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'consoleui.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ConsoleUI_t {
    QByteArrayData data[17];
    char stringdata0[167];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ConsoleUI_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ConsoleUI_t qt_meta_stringdata_ConsoleUI = {
    {
QT_MOC_LITERAL(0, 0, 9), // "ConsoleUI"
QT_MOC_LITERAL(1, 10, 23), // "updateProgressBarSignal"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 8), // "uint64_t"
QT_MOC_LITERAL(4, 44, 3), // "cur"
QT_MOC_LITERAL(5, 48, 5), // "total"
QT_MOC_LITERAL(6, 54, 7), // "success"
QT_MOC_LITERAL(7, 62, 16), // "logMessageSignal"
QT_MOC_LITERAL(8, 79, 12), // "ReplyMsgType"
QT_MOC_LITERAL(9, 92, 4), // "type"
QT_MOC_LITERAL(10, 97, 3), // "msg"
QT_MOC_LITERAL(11, 101, 9), // "readyRead"
QT_MOC_LITERAL(12, 111, 10), // "timerEvent"
QT_MOC_LITERAL(13, 122, 12), // "QTimerEvent*"
QT_MOC_LITERAL(14, 135, 2), // "ev"
QT_MOC_LITERAL(15, 138, 17), // "updateProgressBar"
QT_MOC_LITERAL(16, 156, 10) // "logMessage"

    },
    "ConsoleUI\0updateProgressBarSignal\0\0"
    "uint64_t\0cur\0total\0success\0logMessageSignal\0"
    "ReplyMsgType\0type\0msg\0readyRead\0"
    "timerEvent\0QTimerEvent*\0ev\0updateProgressBar\0"
    "logMessage"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ConsoleUI[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    3,   44,    2, 0x06 /* Public */,
       7,    2,   51,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      11,    0,   56,    2, 0x08 /* Private */,
      12,    1,   57,    2, 0x08 /* Private */,
      15,    3,   60,    2, 0x08 /* Private */,
      16,    2,   67,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 3, QMetaType::Bool,    4,    5,    6,
    QMetaType::Void, 0x80000000 | 8, QMetaType::QString,    9,   10,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 13,   14,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 3, QMetaType::Bool,    4,    5,    6,
    QMetaType::Void, 0x80000000 | 8, QMetaType::QString,    9,   10,

       0        // eod
};

void ConsoleUI::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ConsoleUI *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->updateProgressBarSignal((*reinterpret_cast< uint64_t(*)>(_a[1])),(*reinterpret_cast< uint64_t(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        case 1: _t->logMessageSignal((*reinterpret_cast< ReplyMsgType(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2]))); break;
        case 2: _t->readyRead(); break;
        case 3: _t->timerEvent((*reinterpret_cast< QTimerEvent*(*)>(_a[1]))); break;
        case 4: _t->updateProgressBar((*reinterpret_cast< uint64_t(*)>(_a[1])),(*reinterpret_cast< uint64_t(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        case 5: _t->logMessage((*reinterpret_cast< ReplyMsgType(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ConsoleUI::*)(uint64_t , uint64_t , bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ConsoleUI::updateProgressBarSignal)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ConsoleUI::*)(ReplyMsgType , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ConsoleUI::logMessageSignal)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ConsoleUI::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_ConsoleUI.data,
    qt_meta_data_ConsoleUI,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ConsoleUI::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ConsoleUI::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ConsoleUI.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int ConsoleUI::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void ConsoleUI::updateProgressBarSignal(uint64_t _t1, uint64_t _t2, bool _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ConsoleUI::logMessageSignal(ReplyMsgType _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
