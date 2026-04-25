import random
from datetime import date, datetime, time

from sqlalchemy import (
    CheckConstraint,
    Column,
    Date,
    Float,
    ForeignKey,
    Integer,
    LargeBinary,
    String,
    Time,
    create_engine,
)
from sqlalchemy.dialects.postgresql import ARRAY, JSONB
from sqlalchemy.orm import declarative_base, relationship, sessionmaker

Base = declarative_base()


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

    def __repr__(self):
        return f"<User(id={self.id}, username='{self.username}', email='{self.email}')>"


class MediaStorageImage(Base):
    __tablename__ = "media_storage"

    place_id = Column(Integer, ForeignKey("places.id"))
    id = Column(Integer, primary_key=True)
    filename = Column(String(255))
    extension = Column(String(10))  # To track if it's .pgm, .png, etc.
    image_bytes = Column(LargeBinary)

    map = relationship("Map", back_populates="place")


class MediaStorageFile(Base):
    __tablename__ = "media_storage"

    place_id = Column(Integer, ForeignKey("places.id"))
    id = Column(Integer, primary_key=True)
    filename = Column(String(255))
    extension = Column(String(10))  # To track if it's .pgm, .png, etc.
    metadata_json = Column(JSONB)

    map = relationship("Map", back_populates="place")


class Map(Base):
    __tablename__ = "maps"

    id = Column(Integer, primary_key=True, autoincrement=True)  # integer PRIMARY KEY,
    file_path = Column(String, unique=True)
    place_id = Column(Integer, ForeignKey("places.id"))

    place = relationship("Place", back_populates="map")
    checkpoint = relationship("Checkpoint", back_populates="map")
    alerts = relationship("Alert", back_populates="map")

    def __repr__(self):
        return f"<User(id={self.id}, username='{self.username}', email='{self.email}')>"


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

    alerts = relationship("Alert", back_populates="patrol")
    patrol_link = relationship("PatrolLink", back_populates="patrol")


class PatrolLink(Base):
    __tablename__ = "patrols_link"

    id = Column(Integer, primary_key=True, autoincrement=True)
    patrol_id = Column(String, ForeignKey("patrols.id"))

    patrol = relationship("Patrol", back_populates="patrol_link")


class Checkpoint(Base):
    __tablename__ = "checkpoints"

    id = Column(String, primary_key=True)
    map_id = Column(Integer, ForeignKey("maps.id"))
    x_position = Column(Float)
    y_position = Column(Float)
    yaw = Column(Float)
    gui_yaw = Column(Float)
    status = Column(Integer)
    # You should specify enum values, e.g., Enum('active', 'inactive', name='status_enum'))
    image = Column(String)
    aruco_pose_vector = Column(ARRAY(Float))

    alerts = relationship("Alert", back_populates="checkpoint")
    checkpoint_link = relationship("CheckpointLink", back_populates="checkpoint")
    map = relationship("Map", back_populates="checkpoint")

    calibrations = relationship("Calibration", back_populates="checkpoint_cal")

    __table_args__ = (
        CheckConstraint(
            "array_length(aruco_pose_vector, 1) = 3", name="check_vector_length"
        ),
    )


class CheckpointLink(Base):
    __tablename__ = "checkpoints_link"

    id = Column(Integer, primary_key=True, autoincrement=True)
    checkpoint_id = Column(String, ForeignKey("checkpoints.id"))

    checkpoint = relationship("Checkpoint", back_populates="checkpoint_link")


class Alert(Base):
    __tablename__ = "alerts"

    id = Column(Integer, primary_key=True)
    message = Column(String)
    checkpoint_id = Column(String, ForeignKey("checkpoints.id"))
    patrol_id = Column(String, ForeignKey("patrols.id"))
    x_position = Column(Float)
    y_position = Column(Float)
    yaw = Column(Float)
    camera_data = Column(String)
    lidar_data = Column(String)
    status = Column(Integer)
    date = Column(Date)
    time = Column(Time)
    map_id = Column(Integer, ForeignKey("maps.id"))

    checkpoint = relationship("Checkpoint", back_populates="alerts")
    patrol = relationship("Patrol", back_populates="alerts")
    map = relationship("Map", back_populates="alerts")


class Calibration(Base):
    __tablename__ = "calibrations"

    id = Column(Integer, primary_key=True, autoincrement=True)

    checkpoint_id = Column(String, ForeignKey("checkpoints.id"))
    calibration_value = Column(Float)
    # Defines a column that is an array of Floats
    # The 'dimensions=1' means it's a 1D array (a list)
    calibration_vector = Column(ARRAY(Float))

    checkpoint_cal = relationship("Checkpoint", back_populates="calibrations")

    # To enforce the length of 4, you add a database CHECK constraint
    __table_args__ = (
        CheckConstraint(
            "array_length(calibration_vector, 1) = 8", name="check_vector_length"
        ),
    )


if __name__ == "__main__":
    DATABASE_URL = "postgresql://postgres:123@localhost:5432/postgres"
    engine = create_engine(DATABASE_URL)
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
                camera_data="fuefue",
                lidar_data="fuefue",
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
