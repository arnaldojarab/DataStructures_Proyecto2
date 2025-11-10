from dataclasses import dataclass
@dataclass
class PickupMarker:
    px: int
    py: int
    job_id: str
    expires_at: float

@dataclass
class DropoffMarker:
    px: int
    py: int
    job_id: str
    due_at: float