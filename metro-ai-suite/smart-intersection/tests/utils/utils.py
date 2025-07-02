# SPDX-FileCopyrightText: (C) 2025 Intel Corporation
# SPDX-License-Identifier: LicenseRef-Intel-Edge-Software
# This file is licensed under the Limited Edge Software Distribution License Agreement.

import os
import subprocess
import warnings
import requests
from requests.packages.urllib3.exceptions import InsecureRequestWarning
from selenium.webdriver.support import expected_conditions as EC
from selenium.webdriver.support.ui import WebDriverWait
from selenium.common.exceptions import TimeoutException

def run_command(cmd):
  """Run a shell command and return (stdout, stderr, returncode)."""
  proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  out, err = proc.communicate()
  return out.decode(), err.decode(), proc.returncode

def perform_login(driver, url, username_selector, username_selector_value, password_selector, password_selector_value, login_selector, login_selector_value, username, password):
  """
  Performs login action.

  Args:
    driver: WebDriver instance.
    url: URL of the login page.
    username_selector: Type of selector for the username input (e.g., By.CSS_SELECTOR).
    username_selector_value: Selector value for the username input.
    password_selector: Type of selector for the password input (e.g., By.CSS_SELECTOR).
    password_selector_value: Selector value for the password input.
    login_selector: Type of selector for the login button (e.g., By.CSS_SELECTOR).
    login_selector_value: Selector value for the login button.
    username: Username string.
    password: Password string.
  """
  driver.get(url)  # Load login page

  try:
    # Wait for the username selector to be present
    WebDriverWait(driver, 10).until(EC.presence_of_element_located((username_selector, username_selector_value)))
  except TimeoutException:
    assert False, 'username selector not found within 10 seconds'

  username_input = driver.find_element(username_selector, username_selector_value)
  password_input = driver.find_element(password_selector, password_selector_value)
  login_button = driver.find_element(login_selector, login_selector_value)

  username_input.send_keys(username)
  password_input.send_keys(password)
  login_button.click()  # Try to log in

def read_from_file(file_path):
  """Read content from a specified file."""
  try:
    with open(file_path, 'r') as file:
      content = file.read().strip()
    return content
  except FileNotFoundError:
    print(f"Error: The file '{file_path}' was not found.")
  except IOError:
    print(f"Error: Could not read the file '{file_path}'.")
  return None

def get_password_from_supass_file():
  """Read the password from a supass file."""
  file_path = os.path.join('src', 'secrets', 'supass')
  return read_from_file(file_path)

def get_username_from_influxdb2_admin_username_file():
  """Read the username from a influxdb2-admin-username file."""
  file_path = os.path.join('src', 'secrets', 'influxdb2', 'influxdb2-admin-username')
  return read_from_file(file_path)

def get_password_from_influxdb2_admin_password_file():
  """Read the password from a influxdb2-admin-password file."""
  file_path = os.path.join('src', 'secrets', 'influxdb2', 'influxdb2-admin-password')
  return read_from_file(file_path)

def suppress_insecure_request_warning(func):
  """Decorator to suppress InsecureRequestWarning during test execution."""
  def wrapper(*args, **kwargs):
    # Ignore the InsecureRequestWarning
    warnings.filterwarnings("ignore", category=InsecureRequestWarning)
    try:
      return func(*args, **kwargs)
    finally:
      # Restore the default warning behavior
      warnings.filterwarnings("default", category=InsecureRequestWarning)
  return wrapper

@suppress_insecure_request_warning
def check_components_access(url):
  """Helper function to check if an components is accessible."""
  try:
    # Send a GET request to the URL, ignoring SSL certificate errors
    response = requests.get(url, verify=False)
    
    # Check if the response status code is 200 (OK)
    assert response.status_code == 200, f"Expected status code 200 for {url}, but got {response.status_code}"
  except requests.exceptions.RequestException as e:
    assert False, f"Request to {url} failed: {e}"
