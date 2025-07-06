from sqlalchemy import create_engine, Column, Integer, String, ForeignKey
from sqlalchemy.orm import declarative_base
from sqlalchemy.orm import sessionmaker, relationship

# --- Configuration ---
DATABASE_URL = 'postgresql://postgres:123@localhost:5432/postgres'
# Replace with your PostgreSQL credentials:
# user: your_database_user
# password: your_database_password
# host: your_database_host (e.g., 'localhost' or an IP address)
# port: your_database_port (default is 5432)
# database_name: the name of your database

# --- Define the Base for declarative models ---
Base = declarative_base()

# --- Define Models ---
class User(Base):
    __tablename__ = 'users'

    id = Column(Integer, primary_key=True)
    username = Column(String, unique=True, nullable=False)
    email = Column(String, unique=True, nullable=False)

    # Define a relationship to the Order model
    # back_populates creates a bidirectional relationship
    orders = relationship("Order", back_populates="user")

    def __repr__(self):
        return f"<User(id={self.id}, username='{self.username}', email='{self.email}')>"

class Order(Base):
    __tablename__ = 'orders'

    id = Column(Integer, primary_key=True)
    order_number = Column(String, unique=True, nullable=False)
    user_id = Column(Integer, ForeignKey('users.id'), nullable=False)
    product_name = Column(String, nullable=False)
    amount = Column(Integer, nullable=False)

    # Define a relationship to the User model
    user = relationship("User", back_populates="orders")

    def __repr__(self):
        return (f"<Order(id={self.id}, order_number='{self.order_number}', "
                f"user_id={self.user_id}, product_name='{self.product_name}', "
                f"amount={self.amount})>")

# --- Main Logic ---
def setup_database_and_add_data():
    """
    Sets up the database, creates tables, and adds some sample data.
    """
    engine = create_engine(DATABASE_URL)
    Base.metadata.create_all(engine) # Create tables if they don't exist

    Session = sessionmaker(bind=engine)
    session = Session()

    try:
        # Add users
        user1 = User(username="alice", email="alice@example.com")
        user2 = User(username="bob", email="bob@example.com")
        user3 = User(username="charlie", email="charlie@example.com") # User without orders

        session.add_all([user1, user2, user3])
        session.commit()

        # Add orders
        order1 = Order(order_number="ORD001", user=user1, product_name="Laptop", amount=1200)
        order2 = Order(order_number="ORD002", user=user1, product_name="Mouse", amount=25)
        order3 = Order(order_number="ORD003", user=user2, product_name="Keyboard", amount=75)
        order4 = Order(order_number="ORD004", user=user2, product_name="Monitor", amount=300)

        session.add_all([order1, order2, order3, order4])
        session.commit()
        print("Sample data added successfully!")

    except Exception as e:
        session.rollback()
        print(f"Error setting up data: {e}")
    finally:
        session.close()

def get_data_with_joins():
    """
    Demonstrates different types of joins using SQLAlchemy.
    """
    engine = create_engine(DATABASE_URL)
    Session = sessionmaker(bind=engine)
    session = Session()

    try:
        print("\n--- INNER JOIN: Users with their Orders ---")
        # An inner join returns only the rows where there is a match in both tables.
        # SQLAlchemy's .join() on a relationship implicitly creates an INNER JOIN.
        users_with_orders = session.query(User, Order).join(Order).all()
        for user, order in users_with_orders:
            print(f"User: {user.username}, Order: {order.order_number}, Product: {order.product_name}")

        print("\n--- LEFT OUTER JOIN (or simply LEFT JOIN): All Users and their Orders (if any) ---")
        # A left outer join returns all rows from the left table, and the matched rows from the right table.
        # If there's no match on the right, NULLs are returned for the right table's columns.
        all_users_with_orders = session.query(User, Order).outerjoin(Order).all()
        for user, order in all_users_with_orders:
            order_info = f"Order: {order.order_number}, Product: {order.product_name}" if order else "No Order"
            print(f"User: {user.username}, {order_info}")

        print("\n--- Filtering on Joined Data (e.g., Users who ordered 'Laptop') ---")
        # You can apply filters after the join.
        users_who_ordered_laptop = session.query(User).join(Order).filter(Order.product_name == "Laptop").all()
        for user in users_who_ordered_laptop:
            print(f"User: {user.username} ordered a Laptop.")

        print("\n--- Selecting specific columns from joined tables ---")
        # You can select specific columns instead of entire objects.
        # Use .label() if column names conflict (e.g., both tables have an 'id' column).
        selected_columns = session.query(
            User.username,
            User.email,
            Order.order_number,
            Order.product_name
        ).join(Order).all()
        for username, email, order_number, product_name in selected_columns:
            print(f"User: {username} ({email}), Order: {order_number}, Product: {product_name}")

        print("\n--- Joining without a defined relationship (explicit ON clause) ---")
        # While generally you define relationships, you can also join tables directly
        # by specifying the join condition manually.
        from sqlalchemy import and_
        users_and_orders_explicit_join = session.query(User, Order).join(Order, User.id == Order.user_id).all()
        print("(Same results as implicit INNER JOIN above, just demonstrating explicit 'on' clause)")
        for user, order in users_and_orders_explicit_join:
            print(f"User: {user.username}, Order: {order.order_number}")

    except Exception as e:
        print(f"An error occurred during data retrieval: {e}")
    finally:
        session.close()

if __name__ == "__main__":
    setup_database_and_add_data()
    get_data_with_joins()

