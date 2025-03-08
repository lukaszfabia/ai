from abc import ABC, abstractmethod

from models.graph import Graph


class SearchEngine(ABC):

    def __init__(self, g: Graph):
        self.graph = g

    @abstractmethod
    def search():
        pass
