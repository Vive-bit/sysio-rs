use pyo3::prelude::*;
use pyo3::types::{PyAny, PyDict, PyTuple};

use crate::pywrapper::PyFunctionWrapper;

#[pyclass]
pub struct LruCacheFactory {
    capacity: usize,
}

#[pymethods]
impl LruCacheFactory {
    fn __call__(&self, py: Python, func: PyObject) -> PyResult<PyObject> {
        let wrapper = PyFunctionWrapper::new(py, func, self.capacity)?;
        Py::new(py, wrapper).map(|w| w.into_py(py))
    }

    fn __repr__(&self) -> PyResult<String> {
        Ok(format!("<fastcache.lru_cache factory maxsize={}>", self.capacity))
    }
}

/// Uses:
/// - @lru_cache
/// - @lru_cache(maxsize=256)
/// - decorated = lru_cache(maxsize=256)(func)
#[pyfunction]
#[pyo3(signature = (*args, **kwargs))]
pub fn lru_cache(py: Python, args: &Bound<'_, PyTuple>, kwargs: Option<&Bound<'_, PyDict>>) -> PyResult<PyObject> {
    let maxsize = if let Some(kw) = kwargs {
        if let Some(v) = kw.get_item("maxsize")? {
            v.extract::<usize>()?
        } else if args.len() == 1 && !is_callable(args.get_item(0)?) {
            args.get_item(0)?.extract::<usize>().unwrap_or(128)
        } else {
            128
        }
    } else if args.len() == 1 && !is_callable(args.get_item(0)?) {
        args.get_item(0)?.extract::<usize>().unwrap_or(128)
    } else {
        128
    };

    if args.len() >= 1 && is_callable(args.get_item(0)?) {
        let func: PyObject = args.get_item(0)?.into();
        let wrapper = PyFunctionWrapper::new(py, func, maxsize)?;
        return Py::new(py, wrapper).map(|w| w.into_py(py));
    }

    Py::new(py, LruCacheFactory { capacity: maxsize }).map(|f| f.into_py(py))
}

fn is_callable(obj: &Bound<'_, PyAny>) -> bool {
    obj.is_callable()
}
