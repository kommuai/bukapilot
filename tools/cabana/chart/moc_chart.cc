/****************************************************************************
** Meta object code from reading C++ file 'chart.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "chart.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'chart.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ChartView_t {
    QByteArrayData data[14];
    char stringdata0[159];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ChartView_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ChartView_t qt_meta_stringdata_ChartView = {
    {
QT_MOC_LITERAL(0, 0, 9), // "ChartView"
QT_MOC_LITERAL(1, 10, 22), // "axisYLabelWidthChanged"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 1), // "w"
QT_MOC_LITERAL(4, 36, 13), // "signalUpdated"
QT_MOC_LITERAL(5, 50, 21), // "const cabana::Signal*"
QT_MOC_LITERAL(6, 72, 3), // "sig"
QT_MOC_LITERAL(7, 76, 13), // "manageSignals"
QT_MOC_LITERAL(8, 90, 19), // "handleMarkerClicked"
QT_MOC_LITERAL(9, 110, 10), // "msgUpdated"
QT_MOC_LITERAL(10, 121, 9), // "MessageId"
QT_MOC_LITERAL(11, 131, 2), // "id"
QT_MOC_LITERAL(12, 134, 10), // "msgRemoved"
QT_MOC_LITERAL(13, 145, 13) // "signalRemoved"

    },
    "ChartView\0axisYLabelWidthChanged\0\0w\0"
    "signalUpdated\0const cabana::Signal*\0"
    "sig\0manageSignals\0handleMarkerClicked\0"
    "msgUpdated\0MessageId\0id\0msgRemoved\0"
    "signalRemoved"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ChartView[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    1,   52,    2, 0x08 /* Private */,
       7,    0,   55,    2, 0x08 /* Private */,
       8,    0,   56,    2, 0x08 /* Private */,
       9,    1,   57,    2, 0x08 /* Private */,
      12,    1,   60,    2, 0x08 /* Private */,
      13,    1,   63,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, 0x80000000 | 5,    6,

       0        // eod
};

void ChartView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ChartView *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->axisYLabelWidthChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->signalUpdated((*reinterpret_cast< const cabana::Signal*(*)>(_a[1]))); break;
        case 2: _t->manageSignals(); break;
        case 3: _t->handleMarkerClicked(); break;
        case 4: _t->msgUpdated((*reinterpret_cast< MessageId(*)>(_a[1]))); break;
        case 5: _t->msgRemoved((*reinterpret_cast< MessageId(*)>(_a[1]))); break;
        case 6: _t->signalRemoved((*reinterpret_cast< const cabana::Signal*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 4:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< MessageId >(); break;
            }
            break;
        case 5:
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
            using _t = void (ChartView::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ChartView::axisYLabelWidthChanged)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ChartView::staticMetaObject = { {
    QMetaObject::SuperData::link<QChartView::staticMetaObject>(),
    qt_meta_stringdata_ChartView.data,
    qt_meta_data_ChartView,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ChartView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ChartView::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ChartView.stringdata0))
        return static_cast<void*>(this);
    return QChartView::qt_metacast(_clname);
}

int ChartView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QChartView::qt_metacall(_c, _id, _a);
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
void ChartView::axisYLabelWidthChanged(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
