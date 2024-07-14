/****************************************************************************
** Meta object code from reading C++ file 'k_settings.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "k_settings.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'k_settings.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SettingsWindow_t {
    QByteArrayData data[4];
    char stringdata0[45];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SettingsWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SettingsWindow_t qt_meta_stringdata_SettingsWindow = {
    {
QT_MOC_LITERAL(0, 0, 14), // "SettingsWindow"
QT_MOC_LITERAL(1, 15, 13), // "closeSettings"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 14) // "showDriverView"

    },
    "SettingsWindow\0closeSettings\0\0"
    "showDriverView"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SettingsWindow[] = {

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

void SettingsWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SettingsWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->closeSettings(); break;
        case 1: _t->showDriverView(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (SettingsWindow::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SettingsWindow::closeSettings)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (SettingsWindow::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SettingsWindow::showDriverView)) {
                *result = 1;
                return;
            }
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject SettingsWindow::staticMetaObject = { {
    &QFrame::staticMetaObject,
    qt_meta_stringdata_SettingsWindow.data,
    qt_meta_data_SettingsWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *SettingsWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SettingsWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SettingsWindow.stringdata0))
        return static_cast<void*>(this);
    return QFrame::qt_metacast(_clname);
}

int SettingsWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QFrame::qt_metacall(_c, _id, _a);
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
void SettingsWindow::closeSettings()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void SettingsWindow::showDriverView()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}
struct qt_meta_stringdata_DevicePanel_t {
    QByteArrayData data[6];
    char stringdata0[67];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DevicePanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DevicePanel_t qt_meta_stringdata_DevicePanel = {
    {
QT_MOC_LITERAL(0, 0, 11), // "DevicePanel"
QT_MOC_LITERAL(1, 12, 14), // "showDriverView"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 8), // "poweroff"
QT_MOC_LITERAL(4, 37, 6), // "reboot"
QT_MOC_LITERAL(5, 44, 22) // "updateCalibDescription"

    },
    "DevicePanel\0showDriverView\0\0poweroff\0"
    "reboot\0updateCalibDescription"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DevicePanel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   34,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   35,    2, 0x08 /* Private */,
       4,    0,   36,    2, 0x08 /* Private */,
       5,    0,   37,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void DevicePanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DevicePanel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->showDriverView(); break;
        case 1: _t->poweroff(); break;
        case 2: _t->reboot(); break;
        case 3: _t->updateCalibDescription(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (DevicePanel::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DevicePanel::showDriverView)) {
                *result = 0;
                return;
            }
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject DevicePanel::staticMetaObject = { {
    &ListWidget::staticMetaObject,
    qt_meta_stringdata_DevicePanel.data,
    qt_meta_data_DevicePanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *DevicePanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DevicePanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DevicePanel.stringdata0))
        return static_cast<void*>(this);
    return ListWidget::qt_metacast(_clname);
}

int DevicePanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ListWidget::qt_metacall(_c, _id, _a);
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
void DevicePanel::showDriverView()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
struct qt_meta_stringdata_TogglesPanel_t {
    QByteArrayData data[1];
    char stringdata0[13];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_TogglesPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_TogglesPanel_t qt_meta_stringdata_TogglesPanel = {
    {
QT_MOC_LITERAL(0, 0, 12) // "TogglesPanel"

    },
    "TogglesPanel"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_TogglesPanel[] = {

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

void TogglesPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject TogglesPanel::staticMetaObject = { {
    &ListWidget::staticMetaObject,
    qt_meta_stringdata_TogglesPanel.data,
    qt_meta_data_TogglesPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *TogglesPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *TogglesPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_TogglesPanel.stringdata0))
        return static_cast<void*>(this);
    return ListWidget::qt_metacast(_clname);
}

int TogglesPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ListWidget::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_PersonalisedPanel_t {
    QByteArrayData data[1];
    char stringdata0[18];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PersonalisedPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PersonalisedPanel_t qt_meta_stringdata_PersonalisedPanel = {
    {
QT_MOC_LITERAL(0, 0, 17) // "PersonalisedPanel"

    },
    "PersonalisedPanel"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PersonalisedPanel[] = {

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

void PersonalisedPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject PersonalisedPanel::staticMetaObject = { {
    &ListWidget::staticMetaObject,
    qt_meta_stringdata_PersonalisedPanel.data,
    qt_meta_data_PersonalisedPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *PersonalisedPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PersonalisedPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_PersonalisedPanel.stringdata0))
        return static_cast<void*>(this);
    return ListWidget::qt_metacast(_clname);
}

int PersonalisedPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ListWidget::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_FeaturesControl_t {
    QByteArrayData data[1];
    char stringdata0[16];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_FeaturesControl_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_FeaturesControl_t qt_meta_stringdata_FeaturesControl = {
    {
QT_MOC_LITERAL(0, 0, 15) // "FeaturesControl"

    },
    "FeaturesControl"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_FeaturesControl[] = {

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

void FeaturesControl::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject FeaturesControl::staticMetaObject = { {
    &ButtonControl::staticMetaObject,
    qt_meta_stringdata_FeaturesControl.data,
    qt_meta_data_FeaturesControl,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *FeaturesControl::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *FeaturesControl::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_FeaturesControl.stringdata0))
        return static_cast<void*>(this);
    return ButtonControl::qt_metacast(_clname);
}

int FeaturesControl::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ButtonControl::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_FixFingerprintSelect_t {
    QByteArrayData data[1];
    char stringdata0[21];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_FixFingerprintSelect_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_FixFingerprintSelect_t qt_meta_stringdata_FixFingerprintSelect = {
    {
QT_MOC_LITERAL(0, 0, 20) // "FixFingerprintSelect"

    },
    "FixFingerprintSelect"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_FixFingerprintSelect[] = {

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

void FixFingerprintSelect::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject FixFingerprintSelect::staticMetaObject = { {
    &ButtonControl::staticMetaObject,
    qt_meta_stringdata_FixFingerprintSelect.data,
    qt_meta_data_FixFingerprintSelect,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *FixFingerprintSelect::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *FixFingerprintSelect::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_FixFingerprintSelect.stringdata0))
        return static_cast<void*>(this);
    return ButtonControl::qt_metacast(_clname);
}

int FixFingerprintSelect::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ButtonControl::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_SoftwarePanel_t {
    QByteArrayData data[1];
    char stringdata0[14];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SoftwarePanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SoftwarePanel_t qt_meta_stringdata_SoftwarePanel = {
    {
QT_MOC_LITERAL(0, 0, 13) // "SoftwarePanel"

    },
    "SoftwarePanel"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SoftwarePanel[] = {

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

void SoftwarePanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject SoftwarePanel::staticMetaObject = { {
    &ListWidget::staticMetaObject,
    qt_meta_stringdata_SoftwarePanel.data,
    qt_meta_data_SoftwarePanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *SoftwarePanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SoftwarePanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SoftwarePanel.stringdata0))
        return static_cast<void*>(this);
    return ListWidget::qt_metacast(_clname);
}

int SoftwarePanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ListWidget::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_C2NetworkPanel_t {
    QByteArrayData data[1];
    char stringdata0[15];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_C2NetworkPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_C2NetworkPanel_t qt_meta_stringdata_C2NetworkPanel = {
    {
QT_MOC_LITERAL(0, 0, 14) // "C2NetworkPanel"

    },
    "C2NetworkPanel"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_C2NetworkPanel[] = {

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

void C2NetworkPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject C2NetworkPanel::staticMetaObject = { {
    &ListWidget::staticMetaObject,
    qt_meta_stringdata_C2NetworkPanel.data,
    qt_meta_data_C2NetworkPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *C2NetworkPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *C2NetworkPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_C2NetworkPanel.stringdata0))
        return static_cast<void*>(this);
    return ListWidget::qt_metacast(_clname);
}

int C2NetworkPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ListWidget::qt_metacall(_c, _id, _a);
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
