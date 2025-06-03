import psycopg2
from typing import Optional, List, Tuple
from PyQt5.QtCore import QThread, pyqtSignal, QObject  # , pyqtSlot
from enum import Enum


# cursor.execute("SELECT version();")
# db_version = cursor.fetchone()
# print(f"Database version: {db_version[0]}")


class AlertStatus(Enum):
    ERROR = -100
    WARNING = -50
    INFO = 0


months_abbr = {
    "ene": "enero",
    "feb": "febrero",
    "mar": "marzo",
    "abr": "abril",
    "may": "mayo",
    "jun": "junio",
    "jul": "julio",
    "ago": "agosto",
    "sep": "septiembre",
    "oct": "octubre",
    "nov": "noviembre",
    "dic": "diciembre",
}


class InternalStorageManager:
    def __init__(self) -> None:
        # Database credentials
        self.db_name = "postgres"
        self.db_user = "postgres"
        self.db_password = "123"
        self.db_host = "localhost"
        self.db_port = "5432"

    def update_patrol(self, patrol_data: dict):
        try:
            with psycopg2.connect(
                database=self.db_name,
                user=self.db_user,
                password=self.db_password,
                host=self.db_host,
                port=self.db_port,
            ) as connection:
                with connection.cursor() as cursor:
                    for id, patrol in patrol_data.items():
                        days = list(patrol.get("days").keys())
                        time = list(patrol.get("time"))
                        time = f"{''.join(time[:2])}:{''.join(time[2:])}:00"
                        cursor.execute(
                            f"UPDATE patrol SET time = '{time}', days = '{','.join(days)}'  WHERE id = '{id}';"
                        )
                    connection.commit()  # Commit the transaction
            # return allpatrols
        except psycopg2.Error as e:
            print(f"Database error: {e}")

    def delete_user_patrols(self, ids: List[str]):
        try:
            allpatrols = {}
            with psycopg2.connect(
                database=self.db_name,
                user=self.db_user,
                password=self.db_password,
                host=self.db_host,
                port=self.db_port,
            ) as connection:
                with connection.cursor() as cursor:
                    for id in ids:
                        cursor.execute(
                            f"DELETE FROM patrol_link WHERE patrol_id = '{id}'"
                        )
                    connection.commit()  # Commit the transaction
        except psycopg2.Error as e:
            print(f"Database error: {e}")

    def delete_patrols(self, ids: List[str]):
        try:
            allpatrols = {}
            with psycopg2.connect(
                database=self.db_name,
                user=self.db_user,
                password=self.db_password,
                host=self.db_host,
                port=self.db_port,
            ) as connection:
                with connection.cursor() as cursor:
                    for id in ids:
                        cursor.execute(f"DELETE FROM patrol WHERE id = '{id}'")
                    connection.commit()  # Commit the transaction
            # return allpatrols
        except psycopg2.Error as e:
            print(f"Database error: {e}")

    def get_user_patrols(self):
        pass
        try:
            allpatrols = {}

            with psycopg2.connect(
                database=self.db_name,
                user=self.db_user,
                password=self.db_password,
                host=self.db_host,
                port=self.db_port,
            ) as connection:
                with connection.cursor() as cursor:
                    cursor.execute(
                        "SELECT patrol.id, patrol.time, patrol.days FROM  patrol INNER JOIN patrol_link ON patrol.id  = patrol_link.patrol_id;"
                    )
                    rows = cursor.fetchall()
                    for row in rows:
                        # print(row)
                        id, time, days = row
                        a = {
                            str(id): {
                                "days": {
                                    day: {
                                        "day": day,
                                        "time": time.strftime("%H%M"),
                                        "finished": False,
                                        "patrolid": str(id),
                                    }
                                    for day in days.split(",")
                                },
                                "time": time.strftime("%H%M"),
                            }
                        }
                        allpatrols.update(a)
                        print(a)
                    connection.commit()  # Commit the transaction
            return allpatrols
        except psycopg2.Error as e:
            print(f"Database error: {e}")

    def get_patrols(self):
        pass
        try:
            allpatrols = {}
            with psycopg2.connect(
                database=self.db_name,
                user=self.db_user,
                password=self.db_password,
                host=self.db_host,
                port=self.db_port,
            ) as connection:
                with connection.cursor() as cursor:
                    cursor.execute("SELECT * FROM patrol;")
                    rows = cursor.fetchall()
                    for row in rows:
                        # print(row)
                        id, time, days = row
                        a = {
                            id: {
                                "days": {
                                    day: {
                                        "day": day,
                                        "time": time.strftime("%H%M"),
                                        "finished": False,
                                    }
                                    for day in days.split(",")
                                },
                                "time": time.strftime("%H%M"),
                            }
                        }
                        allpatrols.update(a)
                        print(a)
                    connection.commit()  # Commit the transaction
            return allpatrols
        except psycopg2.Error as e:
            print(f"Database error: {e}")

    def save_patrol(self, patrol_data: dict):
        pass
        try:
            with psycopg2.connect(
                database=self.db_name,
                user=self.db_user,
                password=self.db_password,
                host=self.db_host,
                port=self.db_port,
            ) as connection:
                with connection.cursor() as cursor:
                    for id, patrol in patrol_data.items():
                        days = list(patrol.get("days").keys())
                        time = list(patrol.get("time"))
                        time = f"{''.join(time[:2])}:{''.join(time[2:])}:00"
                        cursor.execute(
                            "INSERT INTO patrol (id, time, days) VALUES (%s, %s, %s);",
                            (id, time, ",".join(days)),
                        )
                        cursor.execute(f"SELECT * FROM patrol WHERE id = '{id}';")
                        rows = cursor.fetchall()
                        for row in rows:
                            print(row)

                        cursor.execute(
                            f"INSERT INTO patrol_link (patrol_id) VALUES ('{id}');",
                        )
                        connection.commit()  # Commit the transaction

        except psycopg2.Error as e:
            print(f"Database error: {e}")

    ####### point
    def save_points(self, points: dict):
        pass
        try:
            with psycopg2.connect(
                database=self.db_name,
                user=self.db_user,
                password=self.db_password,
                host=self.db_host,
                port=self.db_port,
            ) as connection:
                with connection.cursor() as cursor:
                    id1 = list(points.keys())[0]
                    mapfile = points.get(id1).get("mapfile")

                    if mapfile:
                        cursor.execute(
                            f"DELETE FROM point WHERE map_file = '{mapfile}'"
                        )

                    for id, point in points.items():
                        x = point.get("x_meters")
                        y = point.get("y_meters")
                        mapfile = point.get("mapfile")

                        cursor.execute(
                            "INSERT INTO point (id, x_position,  y_position, map_file) VALUES (%s, %s, %s, %s);",
                            (id, x, y, mapfile),
                        )

                        # cursor.execute(
                        #     f"INSERT INTO point_link (point_id) VALUES ('{id}');",
                        # )
                        connection.commit()  # Commit the transaction

        except psycopg2.Error as e:
            print(f"Database error: {e}")

    def get_points(self, mapfile: str):
        pass
        try:
            allpoints = {}
            with psycopg2.connect(
                database=self.db_name,
                user=self.db_user,
                password=self.db_password,
                host=self.db_host,
                port=self.db_port,
            ) as connection:
                with connection.cursor() as cursor:
                    cursor.execute(f"SELECT * FROM point WHERE map_file = '{mapfile}';")
                    rows = cursor.fetchall()
                    allpoints.update({"points": rows})
                    connection.commit()  # Commit the transaction

            # print(allpoints)
            return allpoints
        except psycopg2.Error as e:
            print(f"Database error: {e}")

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
        self, status=None, ascendant=True, page_size=11, page_number=0, map=''
    ):
        try:
            with psycopg2.connect(
                database=self.db_name,
                user=self.db_user,
                password=self.db_password,
                host=self.db_host,
                port=self.db_port,
            ) as connection:
                with connection.cursor() as cursor:
                    if not map:
                        map = ''

                    if status is not None:
                        if ascendant:
                            cursor.execute(
                                f"""
                                           SELECT  x_position, y_position, status, date, time FROM alert
                                           WHERE status = {status} AND  map_file = '{map}'
                                            ORDER BY date ASC
                                            LIMIT {page_size}
                                            OFFSET {page_number* page_size};
                                           """
                            )
                        else:
                            cursor.execute(
                                f"""
                                           SELECT  x_position, y_position, status, date, time FROM alert
                                           WHERE status = {status} AND  map_file = '{map}'
                                            ORDER BY date DESC
                                            LIMIT {page_size}
                                            OFFSET {page_number* page_size};
                                           """
                            )
                    else:
                        if ascendant:
                            cursor.execute(
                                f"""
                                           SELECT  x_position, y_position, status, date, time FROM alert
                                           WHERE  map_file = '{map}'
                                            ORDER BY date ASC
                                            LIMIT {page_size}
                                            OFFSET {page_number* page_size};
                                           """
                            )
                        else:
                            cursor.execute(
                                f"""
                                           SELECT  x_position, y_position, status, date, time FROM alert
                                           WHERE  map_file = '{map}'
                                            ORDER BY date DESC
                                            LIMIT {page_size}
                                            OFFSET {page_number* page_size};
                                           """
                            )

                    rows = cursor.fetchall()
                    connection.commit()  # Commit the transaction

            return rows
        except psycopg2.Error as e:
            print(f"Database error: {e}")

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
                    #retrun in the format  year | month | week_of_month | element_count 

            return rows
        except psycopg2.Error as e:
            print(f"Database error: {e}")

class DataBase(QThread):
    action_completed = pyqtSignal(str, dict)

    def __init__(self, action: str, data: dict = {}) -> None:
        super().__init__()
        self.action = action
        self.data = data

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
        if self.action == "save_points":
            self.internal_storage_manager.save_points(self.data)
            self.action_completed.emit("SuccessSavePoints", {})

        if self.action == "get_points":
            print("DATABASE RUNNING", self.action, self.data)
            data = self.internal_storage_manager.get_points(self.data.get("map_file"))
            self.action_completed.emit("SuccessGetPoinst", data)


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

# The with psycopg2.connect(...) as connection: statement establishes a connection to the PostgreSQL database. When the with block is exited, the connection is automatically closed, even if an exception occurs. The with connection.cursor() as cursor: statement creates a cursor object, which is used to execute SQL queries. The cursor is also automatically closed when the with block is exited.
