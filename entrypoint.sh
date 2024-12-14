#!/bin/bash
set -e

/bin/bash -c "python3 /home/$user/src/rt_cvision/manage.py makemigrations"
/bin/bash -c "python3 /home/$user/src/rt_cvision/manage.py migrate"
# /bin/bash -c "python3 /home/$user/src/rt_cvision/manage.py create_superuser"

# Start supervisord as a background process
sudo -E supervisord -c /etc/supervisord.conf &

# Sleep for a few seconds to ensure supervisord starts properly
echo "Waiting for supervisord to initialize..."
sleep 5

# Start the required services
/bin/bash -c 'supervisorctl start data_acquisition:data_acquisition_server \
                                segmentation:segments_comuting_server \
                                impurity:impurity_comuting_server \
                                visualizer:visualizer_comuting_server'

echo "Sleeping for 10 seconds..."
sleep 10

/bin/bash -c 'supervisorctl start segmentation:segments_data_reader \
                                    impurity:impurity_data_reader \
                                    visualizer:visualizer_data_reader'

echo "Sleeping for 10 seconds..."
sleep 10

/bin/bash -c 'supervisorctl start data_acquisition:data_acquisition_endpoint'

# Keep the container alive by tailing a log file or by sleeping indefinitely
tail -f /dev/null