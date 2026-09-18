# -*- coding: utf-8 -*-

"""Provide road objects classes for the OpenDRIVE implementation."""


class Objects:
    """"""

    def __init__(self):
        self._id = None
        self._type = None
        self._name = None
        self._s = None
        self._t = None
        self._zoffset = None
        self._orientation = None
        self._hdg = None
        self._width = None
        self._length = None
        self._height = None

        self._cornerLocal = []


    @property
    def cornerLocal(self):
        """ """
        return self._cornerLocal

    @property
    def id(self):
        """ """
        return self._id

    @property
    def type(self):
        """ """
        return self._type

    @property
    def name(self):
        """ """
        return self._name

    @property
    def s(self):
        """ """
        return self._s

    @property
    def t(self):
        """ """
        return self._t

    @property
    def zoffset(self):
        """ """
        return self._zoffset

    @property
    def orientation(self):
        """ """
        return self._orientation

    @property
    def hdg(self):
        """ """
        return self._hdg

    @property
    def width(self):
        """ """
        return self._width

    @property
    def length(self):
        """ """
        return self._length

    @property
    def height(self):
        """ """
        return self._height