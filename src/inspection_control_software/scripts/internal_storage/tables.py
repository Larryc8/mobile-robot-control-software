from sqlalchemy import (
    create_engine,
    Column,
    Integer,
    String,
    ForeignKey,
    Date,
    Time,
    Float,
)
from sqlalchemy.orm import declarative_base
from sqlalchemy.orm import sessionmaker, relationship

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


class Map(Base):
    __tablename__ = "maps"
    id = Column(Integer, primary_key=True, autoincrement=True)  # integer PRIMARY KEY,
    file_path = Column(String, unique=True)
    place_id = Column(Integer, ForeignKey("places.id"))

    place = relationship("Place", back_populates="map")
    checkpoint = relationship('Checkpoint', back_populates='map')

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
    patrol_id = Column(String,  ForeignKey("patrols.id"))

    patrol = relationship("Patrol", back_populates="patrol_link")

class Checkpoint(Base):
    __tablename__ = "checkpoints"

    id = Column(String, primary_key=True)
    map_id = Column(Integer, ForeignKey("maps.id"))
    x_position = Column(Float)
    y_position = Column(Float)
    yaw = Column(Float)
    gui_yaw = Column(Float)
    status = Column(Integer)  # You should specify enum values, e.g., Enum('active', 'inactive', name='status_enum'))
    image = Column(String)

    alerts = relationship("Alert", back_populates="checkpoint")
    checkpoint_link = relationship("CheckpointLink", back_populates="checkpoint")
    map = relationship("Map", back_populates="checkpoint")

class CheckpointLink(Base):
    __tablename__ = "checkpoints_link"

    id = Column(Integer, primary_key=True, autoincrement=True)
    checkpoint_id = Column(String, ForeignKey("checkpoints.id"))

    checkpoint = relationship("Checkpoint", back_populates="checkpoint_link")


class Alert(Base):
    __tablename__ = "alerts"

    id = Column(Integer, primary_key=True)
    checkpoint_id = Column(String, ForeignKey("checkpoints.id"))
    patrol_id = Column(String, ForeignKey("patrols.id"))
    x_position = Column(Float)
    y_position = Column(Float)
    yaw = Column(Float)
    camera_data = Column(String)
    lidar_data = Column(String)
    status = Column(Integer)

    checkpoint = relationship("Checkpoint", back_populates="alerts")
    patrol = relationship("Patrol", back_populates="alerts")

if  __name__ == "__main__":
    DATABASE_URL = "postgresql://postgres:123@localhost:5432/postgres"
    engine = create_engine(DATABASE_URL)
    Session = sessionmaker(bind=engine)
    Base.metadata.create_all(engine) # Create tables if they don't exist
    session = Session()
    patrol = Patrol(id='jhfjhsjbfjs', time='12:23:00', days='Lun,Mar')

    session.add(patrol)
    session.commit()
    session = Session()
    patrol = Map(file_path='/dkflfkf/ffrfrfrf/uruuru.yaml')

    session.add(patrol)
    session.commit()



