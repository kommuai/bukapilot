/****************************************************************************
** Meta object code from reading C++ file 'map_renderer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "map_renderer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'map_renderer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MapRenderer_t {
    QByteArrayData data[10];
    char stringdata0[123];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MapRenderer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MapRenderer_t qt_meta_stringdata_MapRenderer = {
    {
QT_MOC_LITERAL(0, 0, 11), // "MapRenderer"
QT_MOC_LITERAL(1, 12, 14), // "updatePosition"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 21), // "QMapLibre::Coordinate"
QT_MOC_LITERAL(4, 50, 8), // "position"
QT_MOC_LITERAL(5, 59, 7), // "bearing"
QT_MOC_LITERAL(6, 67, 11), // "updateRoute"
QT_MOC_LITERAL(7, 79, 21), // "QList<QGeoCoordinate>"
QT_MOC_LITERAL(8, 101, 11), // "coordinates"
QT_MOC_LITERAL(9, 113, 9) // "msgUpdate"

    },
    "MapRenderer\0updatePosition\0\0"
    "QMapLibre::Coordinate\0position\0bearing\0"
    "updateRoute\0QList<QGeoCoordinate>\0"
    "coordinates\0msgUpdate"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MapRenderer[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    2,   29,    2, 0x0a /* Public */,
       6,    1,   34,    2, 0x0a /* Public */,
       9,    0,   37,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3, QMetaType::Float,    4,    5,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void,

       0        // eod
};

void MapRenderer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MapRenderer *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->updatePosition((*reinterpret_cast< QMapLibre::Coordinate(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 1: _t->updateRoute((*reinterpret_cast< QList<QGeoCoordinate>(*)>(_a[1]))); break;
        case 2: _t->msgUpdate(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MapRenderer::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_MapRenderer.data,
    qt_meta_data_MapRenderer,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MapRenderer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MapRenderer::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MapRenderer.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int MapRenderer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
