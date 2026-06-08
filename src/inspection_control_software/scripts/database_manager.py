import os
from datetime import datetime
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np
import psycopg2
import yaml
from internal_storage.tables import (
    Alert,
    Calibration,
    Checkpoint,
    Map,
    Patrol,
    Place,
    patrols_checkpoints_association,
)
from PyQt5.QtCore import QObject, QThread, pyqtSignal  # , pyqtSlot
from sklearn.metrics import mean_squared_error
from sqlalchemy import and_, asc, create_engine, desc, func, insert
from sqlalchemy.orm import declarative_base, sessionmaker

Base = declarative_base()
# cursor.execute("SELECT version();")
# db_version = cursor.fetchone()
# print(f"Database version: {db_version[0]}")


class AlertStatus(Enum):
    ERROR = -100
    WARNING = -50
    INFO = 0


DATABASE_URL = "postgresql://postgres:123@localhost:5432/postgres"


class InternalStorageManager:
    def __init__(self) -> None:
        # Database credentials
        self.db_name = "postgres"
        self.db_user = "postgres"
        self.db_password = "123"
        self.db_host = "localhost"
        self.db_port = "5432"

    def track_inspection(self, data):
        patrol_id = data.get("patrol_id")
        point_id = data.get("point_id")
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        session = Session()

        try:
            # 2. Define the data you want to insert
            new_associations = [
                {
                    "patrol_id": patrol_id,
                    "checkpoint_id": point_id,
                    "date": datetime.now().date(),
                    "time": datetime.now().time(),
                },
            ]

            # 3. Execute the insert statement
            # .execute() automatically handles a list of dictionaries as a bulk insert
            session.execute(insert(patrols_checkpoints_association), new_associations)

            # 4. Commit the transaction to save changes
            session.commit()
            print("Successfully populated the association table!")

        except Exception as e:
            session.rollback()
            print(f"An error occurred: {e}")

        finally:
            session.close()

    def save_places(self, places):
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        session = Session()
        try:
            for place in places:
                name, email, username, institution, phone, address = place
                place = Place(
                    name=name,
                    email=email,
                    username=username,
                    institution=institution,
                    phone=phone,
                    address=address,
                )
                session.add(place)
                session.commit()
        except Exception as e:
            session.rollback()
            print(f"Database error: {e}")
        finally:
            session.close()  # Always close the session

    def get_places(self):
        pass
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        session = Session()
        try:
            places = session.query(Patrol).all()
            session.commit()
            places = [place.name for place in places]
            return places
        except Exception as e:
            session.rollback()
            print(f"Database error: {e}")
        finally:
            session.close()  # Always close the session

    def update_patrol(self, patrols_data: dict):
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        session = Session()
        try:
            for id, patrol in patrols_data.items():
                days = list(patrol.get("days").keys())
                days = ",".join(days)
                time = list(patrol.get("time"))
                time = f"{''.join(time[:2])}:{''.join(time[2:])}:00"
                session.query(Patrol).filter_by(id=id).update(
                    {Patrol.days: days, Patrol.time: time}
                )
                session.commit()
        except Exception as e:
            session.rollback()
            print(f"Database error: {e}")
        finally:
            session.close()  # Always close the session

    def delete_user_patrols(self, ids: List[str]):
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        session = Session()
        try:
            for id in ids:
                session.query(Patrol).filter(Patrol.id == id).update(
                    {Patrol.enabled: False}
                )
                session.commit()

        except Exception as e:
            session.rollback()
            print(f"An error occurred during direct delete: {e}")
        finally:
            session.close()

    def delete_patrols(self, ids: List[str]):
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        session = Session()
        try:
            for id in ids:
                deleted_rows = (
                    session.query(Patrol)
                    .filter(Patrol.id == id)
                    .delete(synchronize_session=False)
                )
                session.commit()

        except Exception as e:
            session.rollback()
            print(f"An error occurred during direct delete: {e}")
        finally:
            session.close()

    def get_user_patrols(self):
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        session = Session()
        try:
            allpatrols = {}
            rows = session.query(Patrol).filter(Patrol.enabled == True)
            for patrol in rows:
                # print(row)
                id, time, days = patrol.id, patrol.time, patrol.days
                formated_time = time.strftime("%H%M")
                _patrol = {
                    str(id): {
                        "days": {
                            day: {
                                "day": day,
                                "time": formated_time,
                                "finished": False,
                                "patrolid": str(id),
                            }
                            for day in days.split(",")
                        },
                        "time": formated_time,
                    }
                }
                allpatrols.update(_patrol)
                # print(a)
            session.commit()  # Commit the transaction
            return allpatrols
        except Exception as e:
            session.rollback()
            print(f"An error occurred during direct delete: {e}")
        finally:
            session.close()

    def get_patrols(self):
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        session = Session()
        try:
            allpatrols = {}
            rows = session.query(Patrol).all()  # cursor.fetchall()
            for patrol in rows:
                id, time, days = patrol.id, patrol.time, patrol.days
                formated_time = time.strftime("%H%M")
                _patrol = {
                    str(id): {
                        "days": {
                            day: {
                                "day": day,
                                "time": formated_time,
                                "finished": False,
                                "patrolid": str(id),
                            }
                            for day in days.split(",")
                        },
                        "time": formated_time,
                    }
                }
                allpatrols.update(_patrol)
            session.commit()  # Commit the transaction
            return allpatrols
        except Exception as e:
            session.rollback()
            print(f"An error occurred during direct delete: {e}")
        finally:
            session.close()

    def save_patrol(self, patrol_data: dict):
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        # Base.metadata.create_all(engine) # Create tables if they don't exist
        session = Session()
        try:
            for id, patrol in patrol_data.items():
                days = list(patrol.get("days").keys())
                time = list(patrol.get("time"))
                time = f"{''.join(time[:2])}:{''.join(time[2:])}:00"
                days = ",".join(days)

                patrol = Patrol(id=id, time=time, days=days, enabled=True)

                session.add(patrol)
                session.commit()
                # patrol_link = PatrolLink(patrol=patrol)
                # session.add(patrol_link)
                # session.commit()
                print("Sample data added successfully!")

        except Exception as e:
            session.rollback()
            print(f"Error setting up data: {e}")
        finally:
            session.close()

    ####### point
    def save_point(self, point: dict):
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        session = Session()

        def get_image_bytes(file_path: str) -> bytes:
            try:
                with open(file_path, "rb") as image_file:
                    return image_file.read()

            except FileNotFoundError:
                print(f"Warning: File not found at {file_path}. Saving without bytes.")
                return b""

        try:
            for id, point_data in point.items():
                mapfile = point_data.get("mapfile")
                mapfile = mapfile.split(".")[0]

                map_db = session.query(Map).filter_by(file_path=mapfile).first()
                session.commit()

                print("DB MAP query, real", mapfile, map_db)

                if map_db:
                    session.add(
                        Checkpoint(
                            id=id,
                            map=map_db,
                            x_position=point_data.get("x_meters"),
                            y_position=point_data.get("y_meters"),
                            yaw=point_data.get("yaw"),
                        )
                    )
                    session.commit()
        except Exception as e:
            session.rollback()
            print(f"Error setting up data: {e}")
        finally:
            session.close()

    def update_point(self, id: str, point: dict):
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        session = Session()
        try:
            map_db = session.query(Checkpoint).filter(Checkpoint.id == id).update(point)
            session.commit()

        except Exception as e:
            session.rollback()
            print(f"Error setting up data: {e}")
        finally:
            session.close()

    def add_points(self, points, mapfile):
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        session = Session()
        try:
            map_db = Map(
                file_path=mapfile
            )  # session.query(Map).filter_by(file_path=mapfile).first()
            # print(f"{__name__} {mapfile}")
            session.add(map_db)
            session.commit()
            print(f"{__name__} point number {len(points)}")

            for id, point in points.items():
                x = point.get("x_meters")
                y = point.get("y_meters")
                mapfile = point.get("mapfile")
                yaw = point.get("yaw")
                image = point.get("image")
                image_content = point.get("image_bytes")
                # aruco_pose = point.get("aruco_pose")

                checkpoint = Checkpoint(
                    id=id,
                    x_position=x,
                    y_position=y,
                    yaw=yaw,
                    map=map_db,
                    image_path=image,
                    image_bytes=image_content,
                )
                session.add(checkpoint)
                session.commit()

        except Exception as e:
            session.rollback()
            print(f"Error setting up data: {e}")
        finally:
            session.close()

    def get_points(self, mapfile: str):
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        session = Session()
        try:
            allpoints = {}
            _points = {}
            # _points = []
            # mapfile = mapfile.split("/")[-1]
            mapfile = mapfile.split(".")[0]
            print(f"{__name__} map {mapfile}")
            map_db = session.query(Map).filter_by(file_path=mapfile).first()
            session.commit()
            if map_db:
                points = (
                    session.query(Checkpoint)
                    .filter(Checkpoint.map_id == map_db.id)
                    .all()
                )
                session.commit()

                for point in points:
                    _points[point.id] = {
                        "id": point.id,
                        "x": point.x_position,
                        "y": point.y_position,
                        "map_file": mapfile,
                        "yaw": point.yaw,
                        "image": point.image_path,
                    }

            allpoints.update({"points": _points})

            return allpoints

        except Exception as e:
            session.rollback()
            print(f"Error setting up data: {e}")
        finally:
            session.close()

    def get_alerts_error(self):
        try:
            with psycopg2.connect(
                database=self.db_name,
                user=self.db_user,
                password=self.db_password,
                host=self.db_host,
                port=self.db_port,
            ) as connection:
                with connection.cursor() as cursor:
                    cursor.execute(
                        "SELECT x_position, y_position, status  FROM alert WHERE date >= CURRENT_DATE - INTERVAL '30 day';"
                    )
                    rows = cursor.fetchall()
                    connection.commit()  # Commit the transaction

            return rows
        except psycopg2.Error as e:
            print(f"Database error: {e}")

    def get_filtered_alerts(
        self, status=None, ascendant=True, page_size=11, page_number=0, map=""
    ):
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        session = Session()

        try:
            if not map:
                map = ""

            map = map.split("/")[-1]
            map = map.split(".")[0]
            print(f"{__name__} map: {map}")

            if status is not None:
                if ascendant:
                    rows = (
                        session.query(Alert, Map)
                        .join(Map)
                        .filter(Map.file_path == map)
                        .order_by(asc(Alert.date))
                        .offset(page_size * page_number)
                        .limit(page_size)
                        .all()
                    )
                    session.commit()
                else:
                    rows = (
                        session.query(Alert, Map)
                        .join(Map)
                        .filter(and_(Map.file_path == map, Alert.status == status))
                        .order_by(desc(Alert.date))
                        .offset(page_size * page_number)
                        .limit(page_size)
                        .all()
                    )
                    session.commit()

            else:
                if ascendant:
                    rows = (
                        session.query(Alert, Map)
                        .join(Map)
                        .filter(Map.file_path == map)
                        .order_by(asc(Alert.date))
                        .offset(page_size * page_number)
                        .limit(page_size)
                        .all()
                    )
                    session.commit()

                else:
                    rows = (
                        session.query(Alert, Map)
                        .join(Map)
                        .filter(Map.file_path == map)
                        .order_by(asc(Alert.date))
                        .offset(page_size * page_number)
                        .limit(page_size)
                        .all()
                    )
            rows = [
                (
                    alert.x_position,
                    alert.y_position,
                    alert.status,
                    alert.date,
                    alert.time,
                    alert.message,
                )
                for alert, map in rows
            ]

            return rows
        except Exception as e:
            session.rollback()
            print(f"Error setting up data: {e}")
        finally:
            session.close()

    def get_alerts_months_stats(self):
        try:
            with psycopg2.connect(
                database=self.db_name,
                user=self.db_user,
                password=self.db_password,
                host=self.db_host,
                port=self.db_port,
            ) as connection:
                with connection.cursor() as cursor:
                    cursor.execute(
                        """
                            SELECT     EXTRACT(MONTH FROM date) AS month,
                            COUNT(*) AS total_count FROM   alert
                            WHERE     EXTRACT(YEAR FROM date) = 2025  AND status = -100
                            GROUP BY     month ORDER BY     month;
                            """
                    )
                    # return in the format  month | total_count
                    rows = cursor.fetchall()
                    connection.commit()  # Commit the transaction

            return rows
        except psycopg2.Error as e:
            print(f"Database error: {e}")

    def get_alerts_weeks_stats(self):
        try:
            with psycopg2.connect(
                database=self.db_name,
                user=self.db_user,
                password=self.db_password,
                host=self.db_host,
                port=self.db_port,
            ) as connection:
                with connection.cursor() as cursor:
                    cursor.execute(
                        """

                        SELECT     EXTRACT(YEAR FROM date) AS year,
                        EXTRACT(MONTH FROM date) AS month,
                        EXTRACT(WEEK FROM date) - EXTRACT(WEEK FROM DATE_TRUNC('month', date)) + 1 AS week_of_month,
                        COUNT(*) AS element_count FROM  alert
                        GROUP BY     year, month, week_of_month ORDER BY     year, month, week_of_month;
                            """
                    )
                    rows = cursor.fetchall()
                    connection.commit()  # Commit the transaction
                    # retrun in the format  year | month | week_of_month | element_count

            return rows
        except psycopg2.Error as e:
            print(f"Database error: {e}")

    def save_alerts(self, alerts: list):
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        session = Session()

        try:
            map_filepath = alerts[0].get("map")
            map_filepath = map_filepath.split("/")[-1]
            map_filepath = map_filepath.split(".")[0]
            map = session.query(Map).filter_by(file_path=map_filepath).first()

            session.commit()

            data = [
                Alert(
                    checkpoint_id=alert.get("heckpoint_id"),
                    patrol_id=alert.get("patrol_id"),
                    message=alert.get("message"),
                    x_position=alert.get("x_position"),
                    y_position=alert.get("x_position"),
                    yaw=alert.get("yaw"),
                    camera_data=alert.get("camera_data"),
                    lidar_data=alert.get("lidar_data"),
                    status=alert.get("status"),
                    date=alert.get("date"),
                    time=alert.get("time"),
                    map=map,
                )
                for alert in alerts
            ]

            session.add_all(data)
            session.commit()
        except Exception as e:
            session.rollback()
            print(f"Error setting up data: {e}")
        finally:
            session.close()

    def save_calibration(self, calibration_data: dict):
        """'
        Save Save Calibrations

        parameter:
        Calibration_data -->
        """
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        # Base.metadata.create_all(engine) # Create tables if they don't exist
        session = Session()
        try:
            calibration_value = calibration_data.get("value")
            checkpoint_id = calibration_data.get("checkpoint_id")
            calibration_vector = None  # calibration_data.get("vector")

            # last_element = session.query(Calibration).filter_by(checkpoint_id=checkpoint_id).order_by(desc(Calibration.id)).first()
            # last_element = (
            #     session.query(Calibration)
            #     .filter_by(checkpoint_id=checkpoint_id)
            #     .order_by(func.random())
            #     .first()
            # )

            # print(
            #     f"Ccalibratonn VEctor {calibration_vector} last_element {last_element}"
            # )
            # calibration_value = None

            # if last_element:
            #     calibration_value = self.loss_func(
            #         calibration_vector, last_element.calibration_vector
            #     )

            calibration = Calibration(
                checkpoint_id=checkpoint_id,
                calibration_value=calibration_value,
                calibration_vector=calibration_vector,
            )

            session.add(calibration)
            session.commit()

            print("Sample data added successfully!")

            session.close()
        except Exception as e:
            session.rollback()
            print(f"Error setting up data: {e}")
        finally:
            session.close()

    def get_calibration(self, pointid: str, current_calibration_vector: list) -> dict:
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        # Base.metadata.create_all(engine) # Create tables if they don't exist
        session = Session()
        try:
            print("--- Running Calculation Query ---")

            # Use sqlalchemy.func to call SQL functions like AVG and STDDEV
            # We query for the average (mean) and standard deviation of the column.
            calculation_query = session.query(
                func.avg(Calibration.calibration_value).label("mean_value"),
                func.stddev(Calibration.calibration_value).label("std_dev_value"),
            ).filter_by(checkpoint_id=pointid)

            # Execute the query and get the single result row
            results = calculation_query.one()

            # random_vector = -1
            # random_row = (
            #     session.query(Calibration)
            #     .order_by(func.random())
            #     .filter_by(checkpoint_id=pointid)
            #     .first()
            # )
            # if random_row:
            #     random_vector = random_row.calibration_vector

            # 6. Display the Results
            mean_val = results.mean_value
            std_dev_val = results.std_dev_value

            print("📊 Statistics for 'calibratio_value':")
            print(f" -> Mean: {mean_val:.4f}")
            print(f" -> Standard Deviation: {std_dev_val:.4f}")
            session.close()
            # return {"std_dev_value": std_dev_val, "mean_value": mean_val, "loss_func": self.loss_func(current_calibration_vector, random_vector)}
            return {"std_dev_value": std_dev_val, "mean_value": mean_val}

        except Exception as e:
            session.rollback()
            session.close()
            print(f"Error setting up data: {e}")
            # return {"std_dev_value": -1, "mean_value": -1, "loss_func": -1}
            return {"std_dev_value": -1, "mean_value": -1}

    def save_ondisk_backup_files(
        self, record_id, output_image_name: str = "", output_yaml_name: str = ""
    ):
        engine = create_engine(DATABASE_URL)
        Session = sessionmaker(bind=engine)
        # Base.metadata.create_all(engine) # Create tables if they don't exist
        session = Session()
        # 1. Fetch the record by ID
        record = session.query(MediaStorageFiles).filter_by(id=record_id).first()

        if not record:
            print(f"No record found with ID: {record_id}")
            return

        # 2. Save the Image (PGM, PNG, etc.)
        # Since we stored it as LargeBinary, we write 'wb' (write binary)
        if output_image_name:
            with open(output_image_name, "wb") as img_file:
                img_file.write(record.image_bytes)

        # 3. Save the YAML
        # We retrieve the JSONB as a Python dict, then dump it to YAML format
        if output_yaml_name:
            with open(output_yaml_name, "w") as yml_file:
                yaml.dump(record.metadata_json, yml_file, default_flow_style=False)

        print(
            f"Success! Saved image to '{output_image_name}' and YAML to '{output_yaml_name}'."
        )

        # Example Usage:
        # save_from_db_to_disk(1, 'restored_image.pgm', 'restored_config.yaml')

    def backup_file(self, image_path: str = "", yaml_path: str = ""):

        # Extract file info
        name = os.path.basename(image_path)
        ext = os.path.splitext(image_path)[1].lower()
        new_file = None
        yaml_data = None
        binary_data = None

        # 1. Read Image (Works for PGM, PNG, JPG, etc.)
        if image_path:
            with open(image_path, "rb") as f:
                binary_data = f.read()

        # 2. Read YAML
        if yaml_path:
            with open(yaml_path, "r") as f:
                yaml_data = yaml.safe_load(f)

        if yaml_data and binary_data:
            new_file = MediaStorageFile(
                filename=name,
                extension=ext,
                image_bytes=binary_data,
                metadata_json=yaml_data,
            )
        else:
            new_file = MediaStorageImage(
                filename=name,
                extension=ext,
                image_bytes=binary_data,
            )

        # 3. Store in Database

        session.add(new_file)
        session.commit()
        print(f"Stored {name} and {yaml_path} successfully.")

    def loss_func(self, x, y):
        # x = np.array(x).reshape(-1, 1)
        # y = np.array(y).reshape(-1, 1)
        # [loss] = mutual_info_regression(x, y)
        loss = mean_squared_error(x, y)
        return loss


class DataBase(QThread):
    action_completed = pyqtSignal(str, dict)

    def __init__(self, action: str, data: dict = {}, map_file="", place="") -> None:
        super().__init__()
        self.action = action
        self.data = data
        self.place = place
        self.map_file = map_file

    def run(self):
        print("DATABASE RUNNING", self.action)
        self.internal_storage_manager = InternalStorageManager()

        if self.action == "save_patrol":
            self.internal_storage_manager.save_patrol(self.data)
            self.action_completed.emit("SuccessSavePatrol", {})
            return

        if self.action == "get_all_patrols":
            data = self.internal_storage_manager.get_patrols()
            self.action_completed.emit("Success", data)
            return

        if self.action == "get_user_patrols":
            data = self.internal_storage_manager.get_user_patrols()
            if not data:
                data = {}
            self.action_completed.emit("SuccessGetAllUserPatrols", data)
            return

        if self.action == "delete_user_patrols":
            data = self.internal_storage_manager.delete_user_patrols(
                self.data.get("ids")
            )
            self.action_completed.emit("Success", {})
            return

        if self.action == "update_patrol":
            self.internal_storage_manager.update_patrol(self.data)
            self.action_completed.emit("Success", {})
            return
        if self.action == "save_point":
            a = self.internal_storage_manager.save_point(self.data)
            self.action_completed.emit("SuccessSavePoints", {"a": a})

        if self.action == "track_inspection":
            a = self.internal_storage_manager.track_inspection(self.data)
            self.action_completed.emit("SuccessTrackInspection", {"a": a})

        if self.action == "update_point":
            for key, value in self.data.items():
                self.internal_storage_manager.update_point(key, value)
            self.action_completed.emit("SuccessUpdatePoint", {})
            return

        if self.action == "add_points":
            self.internal_storage_manager.add_points(self.data, self.map_file)
            self.action_completed.emit("SuccessAddPoints", {})

        if self.action == "get_points":
            print("DATABASE RUNNING", self.action, self.data)
            data = self.internal_storage_manager.get_points(self.data.get("map_file"))
            if not data:
                data = {"points": []}
            self.action_completed.emit("SuccessGetPoinst", data)

        if self.action == "save_alerts":
            print("DATABASE RUNNING", self.action, self.data)
            data = self.internal_storage_manager.save_alerts(self.data.get("alerts"))
            self.action_completed.emit("SuccessSaveAlerts", {})

        if self.action == "save_calibration":
            data = self.internal_storage_manager.save_calibration(self.data)
            self.action_completed.emit("SuccessSaveCalibration", {})
        if self.action == "get_calibration":
            data = self.internal_storage_manager.get_calibration(
                self.data.get("pointid"), self.data.get("current_calibration_vector")
            )
            self.action_completed.emit("SuccessGetCalibration", data)


if __name__ == "__main__":
    patrols_data = {
        "09999990444": {
            "days": {
                "Lun": {"day": "Lun", "time": "20205", "finished": False},
                "Mar": {"day": "Mar", "time": "20202", "finished": False},
                "Mie": {"day": "Mar", "time": "20207", "finished": False},
            },
            "time": "1212",
        }
    }
    e = InternalStorageManager()
    e.save_patrol(patrols_data)
    # e.get_patrols()

# session.query(Patrol).all() The with psycopg2.connect(...) as connection: statement establishes a connection to the PostgreSQL database. When the with block is exited, the connection is automatically closed, even if an exception occurs. The with connection.cursor() as cursor: statement creates a cursor object, which is used to execute SQL queries. The cursor is also automatically closed when the with block is exited.
