#!/bin/bash
set -e

sudo -E supervisord -n -c /etc/supervisord.conf

