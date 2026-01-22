/****************************************************************************
** Meta object code from reading C++ file 'cameraview.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "cameraview.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cameraview.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CameraWidget_t {
    QByteArrayData data[13];
    char stringdata0[214];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CameraWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CameraWidget_t qt_meta_stringdata_CameraWidget = {
    {
QT_MOC_LITERAL(0, 0, 12), // "CameraWidget"
QT_MOC_LITERAL(1, 13, 7), // "clicked"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 19), // "vipcThreadConnected"
QT_MOC_LITERAL(4, 42, 16), // "VisionIpcClient*"
QT_MOC_LITERAL(5, 59, 23), // "vipcThreadFrameReceived"
QT_MOC_LITERAL(6, 83, 27), // "vipcAvailableStreamsUpdated"
QT_MOC_LITERAL(7, 111, 26), // "std::set<VisionStreamType>"
QT_MOC_LITERAL(8, 138, 13), // "vipcConnected"
QT_MOC_LITERAL(9, 152, 11), // "vipc_client"
QT_MOC_LITERAL(10, 164, 17), // "vipcFrameReceived"
QT_MOC_LITERAL(11, 182, 23), // "availableStreamsUpdated"
QT_MOC_LITERAL(12, 206, 7) // "streams"

    },
    "CameraWidget\0clicked\0\0vipcThreadConnected\0"
    "VisionIpcClient*\0vipcThreadFrameReceived\0"
    "vipcAvailableStreamsUpdated\0"
    "std::set<VisionStreamType>\0vipcConnected\0"
    "vipc_client\0vipcFrameReceived\0"
    "availableStreamsUpdated\0streams"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CameraWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   49,    2, 0x06 /* Public */,
       3,    1,   50,    2, 0x06 /* Public */,
       5,    0,   53,    2, 0x06 /* Public */,
       6,    1,   54,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       8,    1,   57,    2, 0x09 /* Protected */,
      10,    0,   60,    2, 0x09 /* Protected */,
      11,    1,   61,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 4,    2,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 7,    2,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 4,    9,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 7,   12,

       0        // eod
};

void CameraWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<CameraWidget *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->clicked(); break;
        case 1: _t->vipcThreadConnected((*reinterpret_cast< VisionIpcClient*(*)>(_a[1]))); break;
        case 2: _t->vipcThreadFrameReceived(); break;
        case 3: _t->vipcAvailableStreamsUpdated((*reinterpret_cast< std::set<VisionStreamType>(*)>(_a[1]))); break;
        case 4: _t->vipcConnected((*reinterpret_cast< VisionIpcClient*(*)>(_a[1]))); break;
        case 5: _t->vipcFrameReceived(); break;
        case 6: _t->availableStreamsUpdated((*reinterpret_cast< std::set<VisionStreamType>(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< std::set<VisionStreamType> >(); break;
            }
            break;
        case 6:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< std::set<VisionStreamType> >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (CameraWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CameraWidget::clicked)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (CameraWidget::*)(VisionIpcClient * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CameraWidget::vipcThreadConnected)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (CameraWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CameraWidget::vipcThreadFrameReceived)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (CameraWidget::*)(std::set<VisionStreamType> );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CameraWidget::vipcAvailableStreamsUpdated)) {
                *result = 3;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CameraWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QOpenGLWidget::staticMetaObject>(),
    qt_meta_stringdata_CameraWidget.data,
    qt_meta_data_CameraWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CameraWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CameraWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CameraWidget.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "QOpenGLFunctions"))
        return static_cast< QOpenGLFunctions*>(this);
    return QOpenGLWidget::qt_metacast(_clname);
}

int CameraWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QOpenGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void CameraWidget::clicked()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void CameraWidget::vipcThreadConnected(VisionIpcClient * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void CameraWidget::vipcThreadFrameReceived()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void CameraWidget::vipcAvailableStreamsUpdated(std::set<VisionStreamType> _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
