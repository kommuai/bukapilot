/****************************************************************************
** Meta object code from reading C++ file 'k_sidebar.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "k_sidebar.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'k_sidebar.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Sidebar_t {
    QByteArrayData data[6];
    char stringdata0[56];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Sidebar_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Sidebar_t qt_meta_stringdata_Sidebar = {
    {
QT_MOC_LITERAL(0, 0, 7), // "Sidebar"
QT_MOC_LITERAL(1, 8, 10), // "openAlerts"
QT_MOC_LITERAL(2, 19, 0), // ""
QT_MOC_LITERAL(3, 20, 12), // "openSettings"
QT_MOC_LITERAL(4, 33, 9), // "openTerms"
QT_MOC_LITERAL(5, 43, 12) // "openTraining"

    },
    "Sidebar\0openAlerts\0\0openSettings\0"
    "openTerms\0openTraining"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Sidebar[] = {

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
       1,    0,   34,    2, 0x06 /* Public */,
       3,    0,   35,    2, 0x06 /* Public */,
       4,    0,   36,    2, 0x06 /* Public */,
       5,    0,   37,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void Sidebar::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Sidebar *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->openAlerts(); break;
        case 1: _t->openSettings(); break;
        case 2: _t->openTerms(); break;
        case 3: _t->openTraining(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Sidebar::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Sidebar::openAlerts)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (Sidebar::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Sidebar::openSettings)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (Sidebar::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Sidebar::openTerms)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (Sidebar::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Sidebar::openTraining)) {
                *result = 3;
                return;
            }
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject Sidebar::staticMetaObject = { {
    &QFrame::staticMetaObject,
    qt_meta_stringdata_Sidebar.data,
    qt_meta_data_Sidebar,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Sidebar::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Sidebar::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Sidebar.stringdata0))
        return static_cast<void*>(this);
    return QFrame::qt_metacast(_clname);
}

int Sidebar::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void Sidebar::openAlerts()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void Sidebar::openSettings()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void Sidebar::openTerms()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void Sidebar::openTraining()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}
struct qt_meta_stringdata_SidebarItem_t {
    QByteArrayData data[1];
    char stringdata0[12];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SidebarItem_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SidebarItem_t qt_meta_stringdata_SidebarItem = {
    {
QT_MOC_LITERAL(0, 0, 11) // "SidebarItem"

    },
    "SidebarItem"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SidebarItem[] = {

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

void SidebarItem::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject SidebarItem::staticMetaObject = { {
    &ClickableWidget::staticMetaObject,
    qt_meta_stringdata_SidebarItem.data,
    qt_meta_data_SidebarItem,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *SidebarItem::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SidebarItem::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SidebarItem.stringdata0))
        return static_cast<void*>(this);
    return ClickableWidget::qt_metacast(_clname);
}

int SidebarItem::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ClickableWidget::qt_metacall(_c, _id, _a);
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
