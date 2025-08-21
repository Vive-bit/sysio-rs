use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict, PyString, PyTuple};

type Key = Arc<[u8]>;

struct Node {
    value: PyObject,
    prev: Option<Key>,
    next: Option<Key>,
}

struct LruCache {
    map: HashMap<Key, Node>,
    head: Option<Key>, // MRU
    tail: Option<Key>, // LRU
    len: usize,
    capacity: usize,
    hits: usize,
    misses: usize,
}

impl LruCache {
    fn new(capacity: usize) -> Self {
        Self {
            map: HashMap::new(),
            head: None,
            tail: None,
            len: 0,
            capacity,
            hits: 0,
            misses: 0,
        }
    }

    fn clear(&mut self) {
        self.map.clear();
        self.head = None;
        self.tail = None;
        self.len = 0;
        self.hits = 0;
        self.misses = 0;
    }

    fn info(&self) -> (usize, usize, usize, usize) {
        (self.hits, self.misses, self.len, self.capacity)
    }

    fn contains_key(&self, key: &Key) -> bool {
        self.map.contains_key(key)
    }

    fn set_capacity(&mut self, new_cap: usize, py: Python) {
        self.capacity = new_cap.max(1);
        while self.len > self.capacity {
            self.evict_lru(py);
        }
    }

    fn get(&mut self, py: Python, key: &Key) -> Option<PyObject> {
        if !self.map.contains_key(key) {
            self.misses += 1;
            return None;
        }
        self.hits += 1;

        let val = self.map.get(key).unwrap().value.clone_ref(py);
        self.touch_front(key);
        Some(val)
    }

    fn put(&mut self, key: Key, value: PyObject, py: Python) {
        if self.map.contains_key(&key) {
            {
                let node = self.map.get_mut(&key).unwrap();
                node.value = value;
            }
            self.touch_front(&key);
            return;
        }

        let node = Node {
            value,
            prev: None,
            next: None,
        };
        self.map.insert(key.clone(), node);
        self.attach_front(&key);
        self.len += 1;

        if self.len > self.capacity {
            self.evict_lru(py);
        }
    }

    fn remove(&mut self, key: &Key) -> bool {
        if !self.map.contains_key(key) {
            return false;
        }

        let (prev, next) = {
            let n = self.map.get(key).unwrap();
            (n.prev.clone(), n.next.clone())
        };

        match prev {
            Some(ref p) => {
                if let Some(node) = self.map.get_mut(p) {
                    node.next = next.clone();
                }
            }
            None => {
                self.head = next.clone();
            }
        }

        match next {
            Some(ref n) => {
                if let Some(node) = self.map.get_mut(n) {
                    node.prev = prev.clone();
                }
            }
            None => {
                self.tail = prev.clone();
            }
        }

        self.map.remove(key);
        self.len -= 1;
        true
    }

    fn evict_lru(&mut self, _py: Python) {
        if let Some(tail_key) = self.tail.clone() {
            self.remove(&tail_key);
        }
    }

    fn touch_front(&mut self, key: &Key) {
        if let Some(ref h) = self.head {
            if Arc::ptr_eq(h, key) {
                return;
            }
        }
        self.detach(key);
        self.attach_front(key);
    }

    fn detach(&mut self, key: &Key) {
        if !self.map.contains_key(key) {
            return;
        }
        let (prev, next) = {
            let n = self.map.get(key).unwrap();
            (n.prev.clone(), n.next.clone())
        };

        match prev {
            Some(ref p) => {
                if let Some(node) = self.map.get_mut(p) {
                    node.next = next.clone();
                }
            }
            None => {
                self.head = next.clone();
            }
        }

        match next {
            Some(ref n) => {
                if let Some(node) = self.map.get_mut(n) {
                    node.prev = prev.clone();
                }
            }
            None => {
                self.tail = prev.clone();
            }
        }

        if let Some(n) = self.map.get_mut(key) {
            n.prev = None;
            n.next = None;
        }
    }

    fn attach_front(&mut self, key: &Key) {
        if let Some(ref h) = self.head {
            if let Some(hn) = self.map.get_mut(h) {
                hn.prev = Some(key.clone());
            }
        }
        if let Some(n) = self.map.get_mut(key) {
            n.prev = None;
            n.next = self.head.clone();
        }
        self.head = Some(key.clone());
        if self.tail.is_none() {
            self.tail = Some(key.clone());
        }
    }
}

#[pyclass]
pub struct PyFunctionWrapper {
    func: PyObject,
    cache: Arc<Mutex<LruCache>>,
    pickle_dumps: PyObject,
    pickle_protocol: i32,
}

#[pymethods]
impl PyFunctionWrapper {
    #[call]
    fn __call__(
        &self,
        py: Python,
        args: &PyTuple,
        kwargs: Option<&PyDict>,
    ) -> PyResult<PyObject> {
        let key_opt = self.make_key(py, args, kwargs);

        if let Ok(ref key) = key_opt {
            if let Some(val) = self.cache.lock().unwrap().get(py, key) {
                return Ok(val);
            }
        }

        let result = self.func.call(py, args, kwargs)?;

        if let Ok(key) = key_opt {
            self.cache
                .lock()
                .unwrap()
                .put(key, result.clone_ref(py), py);
        }
        Ok(result)
    }

    /// Cache leeren.
    fn cache_clear(&self) {
        self.cache.lock().unwrap().clear();
    }

    /// (hits, misses, currsize, maxsize)
    fn cache_info(&self) -> (usize, usize, usize, usize) {
        self.cache.lock().unwrap().info()
    }

    /// Einzelnen Eintrag invalidieren: gleiche Signatur wie Funktionsaufruf.
    #[pyo3(signature = (*args, **kwargs))]
    fn cache_invalidate(&self, py: Python, args: &PyTuple, kwargs: Option<&PyDict>) -> PyResult<bool> {
        match self.make_key(py, args, kwargs) {
            Ok(key) => Ok(self.cache.lock().unwrap().remove(&key)),
            Err(_) => Ok(false),
        }
    }

    /// Prüfen, ob ein Eintrag existiert: gleiche Signatur wie Funktionsaufruf.
    #[pyo3(signature = (*args, **kwargs))]
    fn cache_contains(&self, py: Python, args: &PyTuple, kwargs: Option<&PyDict>) -> PyResult<bool> {
        match self.make_key(py, args, kwargs) {
            Ok(key) => Ok(self.cache.lock().unwrap().contains_key(&key)),
            Err(_) => Ok(false),
        }
    }

    /// Aktuelle maxsize auslesen.
    #[getter]
    fn maxsize(&self) -> usize {
        self.cache.lock().unwrap().capacity
    }

    /// maxsize ändern
    #[setter]
    fn set_maxsize(&self, py: Python, new_size: usize) {
        self.cache.lock().unwrap().set_capacity(new_size, py);
    }

    /// Originale Python-Funktion.
    #[getter]
    fn __wrapped__(&self) -> PyObject {
        self.func.clone()
    }

    fn __repr__(&self, py: Python) -> PyResult<String> {
        let hits_misses = self.cache.lock().unwrap().info();
        let func_repr: String = self.func.as_ref(py).repr()?.extract()?;
        Ok(format!(
            "<fastcache.lru_cache wrapping {} hits={} misses={} size={} maxsize={}>",
            func_repr, hits_misses.0, hits_misses.1, hits_misses.2, hits_misses.3
        ))
    }
}

impl PyFunctionWrapper {
    pub fn new(py: Python, func: PyObject, capacity: usize) -> PyResult<Self> {
        let pickle = pyo3::types::PyModule::import(py, "pickle")?;
        let dumps = pickle.getattr("dumps")?.to_object(py);
        let highest: i32 = pickle.getattr("HIGHEST_PROTOCOL")?.extract()?;
        Ok(Self {
            func,
            cache: Arc::new(Mutex::new(LruCache::new(capacity.max(1)))),
            pickle_dumps: dumps,
            pickle_protocol: highest,
        })
    }

    /// Key für (args, kwargs):
    /// key = pickle.dumps((args, "<KW>", sorted(kwargs.items())))
    /// Falls Pickle fehlschlägt > Fehler (Aufrufer entscheidet über Fallback).
    fn make_key(&self, py: Python, args: &PyTuple, kwargs: Option<&PyDict>) -> PyResult<Key> {
        let mut parts: Vec<PyObject> = Vec::with_capacity(args.len() + 1);
        for a in args.iter() {
            parts.push(a.to_object(py));
        }

        if let Some(kw) = kwargs {
            let mut kv: Vec<(String, PyObject)> = Vec::with_capacity(kw.len());
            for (k, v) in kw.iter() {
                let ks: String = k.extract()?;
                kv.push((ks, v.to_object(py)));
            }
            kv.sort_by(|a, b| a.0.cmp(&b.0));

            parts.push(PyString::new_bound(py, "<KW>").to_object(py));
            for (k, v) in kv.into_iter() {
                parts.push(PyString::new_bound(py, &k).to_object(py));
                parts.push(v);
            }
        }

        let key_tuple = PyTuple::new_bound(py, parts);
        let dumps = self.pickle_dumps.as_ref(py);
        let dumped = dumps.call1((key_tuple, self.pickle_protocol))?;
        let bytes = PyBytes::try_from(dumped)?;
        let vec = bytes.as_bytes().to_vec();
        Ok(Arc::<[u8]>::from(vec.into_boxed_slice()))
    }
}
