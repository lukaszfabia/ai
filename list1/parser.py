from datetime import datetime, time, timedelta
import pandas as pd
from pathlib import Path
from const import DATE_FORMAT
from geopy import Point
from models.graph import Graph, Node, Edge, to_minutes


def minutes_to_hms(minutes: int) -> str:
    hours = minutes // 60
    minutes = minutes % 60
    return (datetime.min + timedelta(hours=hours, minutes=minutes)).strftime("%H:%M:%S")


def add_minutes_to_time(t: time, minutes: int) -> time:
    dummy_date = datetime(1900, 1, 1, t.hour, t.minute)
    new_time = dummy_date + timedelta(minutes=minutes)
    return new_time.time()


def get_df(path: Path = Path("./connection_graph.csv")) -> pd.DataFrame:
    if not path.exists():
        raise FileNotFoundError(f"No file: {path}")

    df = pd.read_csv(
        path.absolute(),
        parse_dates=["departure_time", "arrival_time"],
        date_format=DATE_FORMAT,
        low_memory=False,
        usecols=lambda col: "Unnamed" not in col,
    )

    return df


def to_edge(row) -> Edge:
    s_time = to_minutes(row.departure_time)
    e_time = to_minutes(row.arrival_time)

    estimated_cost = e_time - s_time

    return Edge(
        company=row.company,
        line=row.line,
        departure_time=row.departure_time,
        arrival_time=row.arrival_time,
        start_node=Node(
            name=row.start_stop,
            coords=Point(
                longitude=row.start_stop_lon,
                latitude=row.start_stop_lat,
            ),
            edges=[],
        ),
        end_node=Node(
            name=row.end_stop,
            coords=Point(
                longitude=row.end_stop_lon,
                latitude=row.end_stop_lat,
            ),
            edges=[],
        ),
        cost=estimated_cost,
    )


def to_graph(path: Path = Path("./connection_graph.csv")) -> Graph:
    df = get_df(path=path)

    g = Graph()

    for row in df.itertuples(index=False):
        new_edge = to_edge(row)

        g.add_merged_node(new_edge.start_node)
        g.add_merged_node(new_edge.end_node)
        g.update_edges(new_edge)

    return g
