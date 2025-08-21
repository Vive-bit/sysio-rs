use pyo3::prelude::*;
use pyo3::types::{PyAny, PyDict, PyTuple};

use crate::pywrapper::PyFunctionWrapper;

#[pyclass]
pub struct LruCacheFactory {
    capacity: usize,
}

#[pymethods]
impl LruCacheFactory {
    #[call]
    fn __call__(&self, py: Python, func: PyObject) -> PyResult<PyObject> {
        let wrapper = PyFunctionWrapper::new(py, func, self.capacity)?;
        Py::new(py, wrapper).map(|w| w.into_py(py))
    }

    fn __repr__(&self) -> PyResult<String> {
        Ok(format!("<fastcache.lru_cache factory maxsize={}>", self.capacity))
    }
}

/// - `@lru_cache`
/// - `@lru_cache(maxsize=256)`
/// - `decorated = lru_cache(maxsize=256)(func)`
#[pyfunction]
#[pyo3(signature = (*args, **kwargs))]
pub fn lru_cache(py: Python, args: &PyTuple, kwargs: Option<&PyDict>) -> PyResult<PyObject> {
    // Parameter interpreting
    // 1) args[0] is callable > decoration: lru_cache(func, maxsize=?)
    // 2) no callable in args > Factory: lru_cache(maxsize=?)
    let maxsize = match kwargs.and_then(|k| k.get_item("maxsize")) {
        Some(v) => v.extract::<usize>()?,
        None => {
            if args.len() == 1 && !is_callable(args.get_item(0)) {
                args.get_item(0).extract::<usize>().unwrap_or(128)
            } else {
                128
            }
        }
    };

    if args.len() >= 1 && is_callable(args.get_item(0)) {
        let func: PyObject = args.get_item(0).into();
        let wrapper = PyFunctionWrapper::new(py, func, maxsize)?;
        return Py::new(py, wrapper).map(|w| w.into_py(py));
    }

    Py::new(py, LruCacheFactory { capacity: maxsize }).map(|f| f.into_py(py))
}

fn is_callable(obj: &PyAny) -> bool {
    obj.is_callable()
}
