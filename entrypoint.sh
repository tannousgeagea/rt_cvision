#!/bin/bash
set -e

# Define the user
USER=$user

echo "🔄 Running Django Migrations..."
/bin/bash -c "python3 /home/$USER/src/rt_cvision/manage.py makemigrations"
/bin/bash -c "python3 /home/$USER/src/rt_cvision/manage.py migrate"
/bin/bash -c "python3 /home/$USER/src/rt_cvision/manage.py create_superuser"

# ✅ Detect ROS2 Topics Before Starting Services
echo "🔍 Detecting ROS2 Topics..."
/bin/bash -c "source /opt/ros/humble/setup.bash && python3 /home/$USER/src/rt_cvision/manage.py detect_ros2_topics"

# ✅ Start Supervisor (including Django) immediately so admin UI is available
echo "🚀 Starting Supervisor (Django will be available)..."
sudo -E supervisord -c /etc/supervisord.conf &

# Sleep for a few seconds to ensure Django starts properly
echo "⏳ Waiting for Django to initialize..."
sleep 5

# 🚨 Wait for configuration before starting other services
echo "🚨 Waiting for app configuration before launching other services..."
until /bin/bash -c "python3 /home/$USER/src/rt_cvision/manage.py check_config"; do
    echo "🔄 Configuration not found. Waiting..."
    sleep 5
done
echo "✅ App is configured! Proceeding..."

# 🚀 Start delayed services after configuration is complete
echo "🚀 Starting Core Services..."
/bin/bash -c 'supervisorctl start data_acquisition:data_acquisition_server \
                                segmentation:segments_comuting_server \
                                impurity:impurity_comuting_server \
                                visualizer:visualizer_comuting_server'

echo "⏳ Sleeping for 10 seconds..."
sleep 10

/bin/bash -c 'supervisorctl start segmentation:segments_data_reader \
                                    impurity:impurity_data_reader \
                                    visualizer:visualizer_data_reader'

echo "⏳ Sleeping for 10 seconds..."
sleep 10

/bin/bash -c 'supervisorctl start data_acquisition:data_acquisition_endpoint'

# Keep the container alive by tailing a log file or by sleeping indefinitely
echo "✅ All services started. Keeping container alive..."
tail -f /dev/null
