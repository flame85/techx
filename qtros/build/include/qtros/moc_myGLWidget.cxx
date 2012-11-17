/****************************************************************************
** Meta object code from reading C++ file 'myGLWidget.hpp'
**
** Created: Sat Nov 17 14:30:07 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/qtros/myGLWidget.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'myGLWidget.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MyGLWidget[] = {

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
      12,   11,   11,   11, 0x0a,
      39,   29,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_MyGLWidget[] = {
    "MyGLWidget\0\0resetRobotPose()\0transform\0"
    "addTransform(QMatrix4x4)\0"
};

const QMetaObject MyGLWidget::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_MyGLWidget,
      qt_meta_data_MyGLWidget, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MyGLWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MyGLWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MyGLWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MyGLWidget))
        return static_cast<void*>(const_cast< MyGLWidget*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int MyGLWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: resetRobotPose(); break;
        case 1: addTransform((*reinterpret_cast< QMatrix4x4(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
