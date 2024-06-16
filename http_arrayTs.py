import mysql.connector
from flask import Flask, request, jsonify
from datetime import datetime
import socket
import ssl

app = Flask(__name__)
DB_CONFIG = {
    'host': '127.0.0.1',
    'user': '',
    'password': '',
    'database': 'pee51',
    'ssl_cert': '/home/user/Documents/pee51_client_certificate.pem',
    'ssl_key' : '/home/user/Documents/pee51_client_key.pem',
    'ssl_ca'  : '/home/user/Documents/ca_certificate.pem'
}

def get_db_connection():
    connection = getattr(Flask, '_connection', None)
    if connection is None:
        connection = mysql.connector.connect(**DB_CONFIG)
        Flask._connection = connection
    return connection

def close_db_connection():
    connection = getattr(Flask, '_connection', None)
    if connection is not None:
        connection.close()
        del Flask._connection

def insert_telemetry_data(values):
    try:
        connection = get_db_connection()
        cursor = connection.cursor()

        # Determine the maximum number of measurements
        max_measurements = max(len(values[key]) for key in values)

        for i in range(max_measurements):
            query = "INSERT INTO pee51 (Temperatuur_gas, Zuurtegraad, Stroom, Spanning, Temp1, Temp2, Temp3, Temp4, Temp5, Luchtvochtigheid, Geleidbaarheid, Flowsensor, Flowsensor2, CO, Waterstof, ts) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
            #ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            #ts = values.get('ts')[i] if values.get('ts') and len(values.get('ts')) > i else None #ts = values.get('ts')[i] if len(values.get('ts')) > i else None
            data_tuple = (
                values.get('Temperatuur_gas')[i] if len(values.get('Temperatuur_gas')) and len(values.get('Temperatuur_gas')) > i else None,
                values.get('Zuurtegraad')[i] if len(values.get('Zuurtegraad')) and len(values.get('Zuurtegraad')) > i else None,
                values.get('Stroom')[i] if len(values.get('Stroom')) and len(values.get('Stroom')) > i else None,
                values.get('Spanning')[i] if len(values.get('Spanning')) and len(values.get('Spanning')) > i else None,
                values.get('Temp1')[i] if len(values.get('Temp1')) and len(values.get('Temp1')) > i else None,
                values.get('Temp2')[i] if len(values.get('Temp2')) and len(values.get('Temp2')) > i else None,
                values.get('Temp3')[i] if len(values.get('Temp3')) and len(values.get('Temp3')) > i else None,
                values.get('Temp4')[i] if len(values.get('Temp4')) and len(values.get('Temp4')) > i else None,
                values.get('Temp5')[i] if len(values.get('Temp5')) and len(values.get('Temp5')) > i else None,
                values.get('Luchtvochtigheid')[i] if len(values.get('Luchtvochtigheid')) and len(values.get('Luchtvochtigheid')) > i else None,
                values.get('Geleidbaarheid')[i] if len(values.get('Geleidbaarheid')) and len(values.get('Geleidbaarheid')) > i else None,
                values.get('Flowsensor')[i] if len(values.get('Flowsensor')) and len(values.get('Flowsensor')) > i else None,
                values.get('Flowsensor2')[i] if len(values.get('Flowsensor2')) and len(values.get('Flowsensor2')) > i else None,
                values.get('CO')[i] if len(values.get('CO')) and len(values.get('CO')) > i else None,
                values.get('Waterstof')[i] if len(values.get('Waterstof')) and len(values.get('Waterstof')) > i else None,
                values.get('ts')[i] if values.get('ts') and len(values.get('ts')) > i else None
            )
            cursor.execute(query, data_tuple)
            connection.commit()

        print("Telemetry data inserted successfully")
    except Exception as e:
        print("Error inserting telemetry data:", e)


@app.route('/api/telemetry', methods=['POST'])
def log_telemetry():
    try:
        data = request.json.get('values')
        print("Received data:", data)  # Debugging statement
        insert_telemetry_data(data)
        return jsonify({"message": "Telemetry logged successfully"}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.teardown_appcontext
def teardown_db(exception):
    close_db_connection()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
