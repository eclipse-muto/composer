from dataclasses import dataclass
from typing import Optional


@dataclass()
class Difference:
    added_nodes: Optional[dict]
    removed_nodes: Optional[dict]
    common_nodes: Optional[dict]
