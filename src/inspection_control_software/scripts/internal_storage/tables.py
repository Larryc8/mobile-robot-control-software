import random
from datetime import date, datetime, time
from typing import List

from sqlalchemy import (
    Boolean,
    CheckConstraint,
    Column,
    Date,
    Float,
    ForeignKey,
    Integer,
    LargeBinary,
    String,
    Table,
    Time,
    create_engine,
)
from sqlalchemy.dialects.postgresql import ARRAY, JSONB
from sqlalchemy.orm import Mapped, declarative_base, relationship, sessionmaker

Base = declarative_base()


patrols_checkpoints_association = Table(
    "patrols_checkpoints",
    Base.metadata,
    Column("patrol_id", ForeignKey("patrols.id"), primary_key=True),
    Column("checkpoint_id", ForeignKey("checkpoints.id"), primary_key=True),
    Column("date", Date),
    Column("time", Time),
)


class Place(Base):
    __tablename__ = "places"

    id = Column(Integer, primary_key=True, autoincrement=True)
    username = Column(String, unique=True, nullable=False)
    email = Column(String, unique=True, nullable=False)
    name = Column(Integer)  # integer,
    address = Column(String)  # varchar,
    institution = Column(String)  # varchar,
    phone = Column(String)  # varchar

    map = relationship("Map", back_populates="place")


class Map(Base):
    __tablename__ = "maps"

    id = Column(Integer, primary_key=True, autoincrement=True)  # integer PRIMARY KEY,
    file_path = Column(String, unique=True)
    place_id = Column(Integer, ForeignKey("places.id"))
    image_bytes = Column(LargeBinary)
    metadata_json = Column(JSONB)

    place = relationship("Place", back_populates="map")
    checkpoint = relationship("Checkpoint", back_populates="map")
    alerts = relationship("Alert", back_populates="map")


class Patrol(Base):
    __tablename__ = "patrols"

    id = Column(String, primary_key=True)  # varchar PRIMARY KEY,
    place_id = Column(Integer)  # integer,
    time = Column(Time)
    days = Column(String)
    start_time = Column(Time)
    start_date = Column(Date)
    end_time = Column(Time)
    end_date = Column(Date)
    enabled = Column(Boolean, default=True)

    alerts = relationship("Alert", back_populates="patrol")
    checkpoints: Mapped[List["Checkpoint"]] = relationship(
        secondary=patrols_checkpoints_association, back_populates="patrols"
    )


class Checkpoint(Base):
    __tablename__ = "checkpoints"

    id = Column(String, primary_key=True)
    map_id = Column(Integer, ForeignKey("maps.id"))
    x_position = Column(Float)
    y_position = Column(Float)
    yaw = Column(Float)
    status = Column(Integer)
    image_path = Column(String)
    image_bytes = Column(LargeBinary)
    is_home = Column(Boolean, default=False)
    enabled = Column(Boolean, default=True)

    alerts = relationship("Alert", back_populates="checkpoint")
    map = relationship("Map", back_populates="checkpoint")
    calibrations = relationship("Calibration", back_populates="checkpoint_cal")
    # The relationship pointing to Course, using the association table
    patrols: Mapped[List["Patrol"]] = relationship(
        secondary=patrols_checkpoints_association, back_populates="checkpoints"
    )


class Alert(Base):
    __tablename__ = "alerts"

    id = Column(Integer, primary_key=True)
    message = Column(String)
    x_position = Column(Float)
    y_position = Column(Float)
    yaw = Column(Float)
    status = Column(Integer)
    date = Column(Date)
    time = Column(Time)

    checkpoint_id = Column(String, ForeignKey("checkpoints.id"))
    patrol_id = Column(String, ForeignKey("patrols.id"))
    map_id = Column(Integer, ForeignKey("maps.id"))

    checkpoint = relationship("Checkpoint", back_populates="alerts")
    patrol = relationship("Patrol", back_populates="alerts")
    map = relationship("Map", back_populates="alerts")


class Calibration(Base):
    __tablename__ = "calibrations"

    id = Column(Integer, primary_key=True, autoincrement=True)
    checkpoint_id = Column(String, ForeignKey("checkpoints.id"))
    calibration_value = Column(Float)

    checkpoint_cal = relationship("Checkpoint", back_populates="calibrations")


if __name__ == "__main__":
    from sqlalchemy import MetaData

    def drop_everything(engine):
        metadata = MetaData()
        # Reflect looks at the database and "discovers" all existing tables
        metadata.reflect(bind=engine)

        # Drop all discovered tables
        metadata.drop_all(bind=engine)

    DATABASE_URL = "postgresql://postgres:123@localhost:5432/postgres"
    engine = create_engine(DATABASE_URL)
    drop_everything(engine)
    Session = sessionmaker(bind=engine)
    Base.metadata.create_all(engine)  # Create tables if they don't exist
    session = Session()
    patrol = Patrol(id="jhfjhsjbfjs", time="12:23:00", days="Lun,Mar")

    session.add(patrol)
    session.commit()
    # session = Session()
    map = Map(
        file_path="/pico-sdk/mobile-robot-control-software/src/inspection_control_software/scripts/mymap.yaml"
    )

    session.add(map)
    session.commit()

    status_options = [-100, -50, 0]  # 0=active, 1=resolved, 2=ignored
    checkpoint_ids = [f"cp_{i}" for i in range(1, 11)]  # 10 checkpoints
    patrol_ids = [f"patrol_{i}" for i in range(1, 6)]  # 5 patrols
    map_ids = [1, 2, 3]  # 3 maps

    alerts = []

    try:
        for i in range(100):
            alert = Alert(
                # checkpoint_id=random.choice(checkpoint_ids),
                # patrol_id=random.choice(patrol_ids),
                message=random.choice(["Error", "Advertencia", "Info"]),
                x_position=random.uniform(-1.3, 1.3),
                y_position=random.uniform(-1.3, 1.3),
                yaw=random.uniform(-1.6, 1.6),
                # camera_data="fuefue",
                # lidar_data="fuefue",
                status=random.choice(status_options),
                date=date.today(),
                time=time(
                    hour=random.randint(0, 23),
                    minute=random.randint(0, 59),
                    second=random.randint(0, 59),
                ),
                map=map,
            )
            alerts.append(alert)

        session.add_all(alerts)
        session.commit()
    except Exception as e:
        session.rollback()
        print(f"An error occurred: {e}")
    finally:
        session.close()
