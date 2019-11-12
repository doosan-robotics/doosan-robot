import boto3
import json
import sys
from time import gmtime, strftime, sleep
import random
import subprocess
import os
import yaml
import re

SETTING_FILE="./ws_settings.yaml"
SETTINGS = ["aws_region", "bucket_name", 
            "simulation_app_name", 
            "robot_app_name", 
            "default_vpc", 
            "security_groups", 
            "subnets", 
            "iam_policy", 
            "iam_role", 
            "iam_role_for_deployment", 
            "robomaker_settings"]
            
class Setup:
    def __init__(self):
        self.settings = {}

    def setup_robomaker_settings(self):
        log("setup roboMakerSettings.json..")

        try:
            file_name = "roboMakerSettings.json" 
            with open(file_name) as f:
                lines = f.read()

            for item in self.settings:
                value = str(self.settings[item])
                value = value.replace("'", "\"")
                lines = lines.replace("<{}>".format(item), value)

            with open(file_name, mode="w") as f:
                f.write(lines)        
        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None

        return True

    def setup_simulation_app_name(self):
        try:
            id = boto3.client('sts').get_caller_identity()
            arn = id["Arn"]
            i = arn.rfind('/')
            if i > 0:
                name = re.sub(r'[\.,@]+', "-", arn[i+1:])
                result = "doosan_robot_sumulation_{}".format(name)
            else:
                result = "doosan_robot_sumulation"
        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None

        return result
        
    def setup_robot_app_name(self):
        try:
            id = boto3.client('sts').get_caller_identity()
            arn = id["Arn"]
            i = arn.rfind('/')
            if i > 0:
                name = re.sub(r'[\.,@]+', "-", arn[i+1:])
                result = "doosan_robot_robotapp_{}".format(name)
            else:
                result = "doosan_robot_robotapp_"
        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None

        return result
            

    def setup_aws_region(self):
        try:
            session = boto3.session.Session()
            result = session.region_name
        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None

        if result not in ["us-west-2", "us-east-1", "eu-west-1", "ap-northeast-1"]:
            errlog("This material is only avaiable in  US East (N. Virginia), US West (Oregon), EU (Ireland) and Asia Pacific (Tokyo) now.\n\n")
            return None
        
        return result

    def setup_default_vpc(self):
        try:
            ec2 = boto3.client('ec2')
            result = [vpc['VpcId'] for vpc in ec2.describe_vpcs()['Vpcs'] if vpc["IsDefault"] == True][0]
        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None

        return result

    def setup_security_groups(self):
        try:
            ec2 = boto3.client('ec2')
            result = [group["GroupId"] for group in ec2.describe_security_groups()['SecurityGroups'] \
                if 'VpcId' in group and group["GroupName"] == "default" and group["VpcId"] == self.settings["default_vpc"]]
        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None

        return result
        
    def setup_subnets(self):
        try:
            ec2 = boto3.client('ec2')
            result = [subnet["SubnetId"] for subnet in ec2.describe_subnets()["Subnets"] \
                      if subnet["VpcId"] == self.settings["default_vpc"] and subnet['DefaultForAz']==True]
        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None
            
        return result

    def setup_bucket_name(self):
        done = False
        retry = 5
        try:
            id = boto3.client('sts').get_caller_identity()
            accountId = id["Account"]
        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None
            
        while(not done):
            tm = gmtime()
            result = "robomaker-ws-%s-%s-%s" % (self.settings["aws_region"], accountId,strftime("%y%m%d-%H%M%S", tm))  
            
            log("Create S3 bucket: %s.." % result)
            #create bucket 
            try: 
                region_name = self.settings["aws_region"]
                s3_client = boto3.client('s3', region_name=region_name)
                if region_name == "us-east-1":
                    s3_client.create_bucket(Bucket=result)
                else:
                    location = {'LocationConstraint': self.settings["aws_region"]}
                    s3_client.create_bucket(Bucket=result, CreateBucketConfiguration=location)
            except Exception as e:
                retry -= 1
                if retry >= 0:
                    log(" => Failed.. retrying")
                    sleep(random.randint(1,3))
                    continue
                else:
                    errlog("Failed to create S3 bucket!")
                    errlog("Error Message: %s" % str(e))
                    return None
            done = True
            
        return result
        
    def setup_iam_policy(self):
        log("create iam policy..")
        iam = boto3.client('iam')
        
        tm = gmtime()
        policy_name = "robomaker-ws-policy-%s" % (strftime("%y%m%d-%H%M%S", tm)) 
        log("policy name : %s" % policy_name)

        my_policy = {
            "Version": "2012-10-17",
            "Statement": [
                {
                    "Effect": "Allow",
                    "Action": [
                        "cloudwatch:PutMetricData",
                        "logs:CreateLogGroup",
                        "logs:CreateLogStream",
                        "logs:PutLogEvents",
                        "logs:DescribeLogStreams",
                    ],
                    "Resource": "*"
                },
                {
                    "Effect": "Allow",
                    "Action": [
                        "robomaker:UpdateRobotDeployment"
                    ],
                    "Resource": "*"
                },
                {
                    "Effect": "Allow",
                    "Action": [
                        "s3:Get*",
                        "s3:List*",
                        "s3:Put*",
                        "s3:DeleteObject"
                    ],
                    "Resource": ["arn:aws:s3:::{}/*".format(self.settings["bucket_name"])]
                }
            ]
        }
        
        try:
            response = iam.create_policy(
              PolicyName=policy_name,
              PolicyDocument=json.dumps(my_policy)
            )
            result = response["Policy"]["Arn"]
        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None
            
        return result

    def setup_iam_role(self):
        log("create iam role..")
        iam = boto3.client('iam')
        path='/'
        tm = gmtime()
        role_name = "Cloud9-robomaker-ws-role-%s" % (strftime("%y%m%d-%H%M%S", tm)) 
        log("role name : %s" % role_name)
        description='Role for RoboMaker'

        trust_policy={
          "Version": "2012-10-17",
          "Statement": [
            {
              "Sid": "",
              "Effect": "Allow",
              "Principal": {
                "Service": "robomaker.amazonaws.com"
              },
              "Action": "sts:AssumeRole"
            }
          ]
        }
        
        tags=[
            {
                'Key': 'Environment',
                'Value': 'Test'
            }
        ]
        
        try:
            response = iam.create_role(
                Path=path,
                RoleName=role_name,
                AssumeRolePolicyDocument=json.dumps(trust_policy),
                Description=description,
                MaxSessionDuration=3600,
                Tags=tags
            )

            result = response["Role"]["Arn"]

            iam.attach_role_policy(
                RoleName= role_name,
                PolicyArn= self.settings["iam_policy"]
            )

        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None
            
        return result
            
    def setup_iam_role_for_deployment(self):
        log("create iam role for deployment..")
        iam = boto3.client('iam')
        path='/'
        tm = gmtime()
        role_name = "Cloud9-robomaker-ws-deployment-role-%s" % (strftime("%y%m%d-%H%M%S", tm)) 
        log("deployment role name : %s" % role_name)
        description='Role for RoboMaker - deployment'

        trust_policy={
          "Version": "2012-10-17",
          "Statement": [
            {
              "Sid": "",
              "Effect": "Allow",
              "Principal": {
                "Service": [
                    "lambda.amazonaws.com",
                    "greengrass.amazonaws.com"
                ]
              },
              "Action": "sts:AssumeRole"
            }
          ]
        }
        
        tags=[
            {
                'Key': 'Environment',
                'Value': 'Test'
            }
        ]
        
        try:
            response = iam.create_role(
                Path=path,
                RoleName=role_name,
                AssumeRolePolicyDocument=json.dumps(trust_policy),
                Description=description,
                MaxSessionDuration=3600,
                Tags=tags
            )

            result = response["Role"]["Arn"]

            iam.attach_role_policy(
                RoleName= role_name,
                PolicyArn= self.settings["iam_policy"]
            )

        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None
            
        return result


    def postProcess(self):
        try:
            log("Perform the build...")
            log("Setp 1. simulation app - install dependencies...")
            os.chdir(os.path.abspath('./simulation_ws'))
            result = subprocess.call("rosdep install --from-paths src --ignore-src -r -y".split())
            if result == 0:
                log(" =>OK")
            else:
                errlog("Setup failed!")
                errlog("Reason: Failed to bundle the sample program!")
                sys.exit(1)
                
            log("Setp 2. simulation app - build...")
            result = subprocess.call("colcon build".split())
            if result == 0:
                log(" =>OK")
            else:
                errlog("Setup failed!")
                errlog("Reason: Failed to build the sample program!")
                sys.exit(1)
                
            log("Setp 3. simulation app - bundle...")
            result = subprocess.call("colcon bundle".split())
            if result == 0:
                log(" =>OK")
            else:
                errlog("Setup failed!")
                errlog("Reason: Failed to bundle the sample program!")
                sys.exit(1)

            log("Setp 4. robot app - install dependencies...")
            os.chdir(os.path.abspath('../robot_ws'))
            result = subprocess.call("rosdep install --from-paths src --ignore-src -r -y".split())
            if result == 0:
                log(" =>OK")
            else:
                errlog("Setup failed!")
                errlog("Reason: Failed to bundle the sample program!")
                sys.exit(1)
                
            log("Setp 5. robot app - build...")
            result = subprocess.call("colcon build".split())
            if result == 0:
                log(" =>OK")
            else:
                errlog("Setup failed!")
                errlog("Reason: Failed to build the sample program!")
                sys.exit(1)
                
            log("Setp 6. robot app - bundle...")
            result = subprocess.call("colcon bundle".split())
            if result == 0:
                log(" =>OK")
            else:
                errlog("Setup failed!")
                errlog("Reason: Failed to bundle the sample program!")
                sys.exit(1)
                
        except Exception as e:
            errlog("Exception : %s" % str(e))
            sys.exit(1)

    def saveSettings(self):
        for aSetting in SETTINGS:
            if not aSetting in self.settings:
                self.settings[aSetting] = None
        
        f = open(SETTING_FILE, "w+")
        f.write(yaml.dump(self.settings, default_flow_style=False))
    
    def entry(self):
        if os.path.exists(SETTING_FILE):
            try:
                f = open(SETTING_FILE, "r+")
                self.settings = yaml.load(f)
                if not self.settings:
                    self.settings = {}
            except Exception as e:
                errlog("\nSetup failed! \n => Reason: Setting file %s exists but failed to load\n" % SETTING_FILE)
                errlog(" => Error Message: %s\n\n" % str(e))
                sys.exit(1)
        else:
            self.settings = {}

        for aSetting in SETTINGS:
            print("Check %s" % aSetting)
            if (not aSetting in self.settings) or (not self.settings[aSetting]) :
                func_name = "setup_%s" % aSetting
                result = getattr(self, func_name)()
                if not result:
                    errlog("Failed to setup %s\nFinishing...\n" % aSetting)
                    self.saveSettings()
                    return

                self.settings[aSetting] = result
                
            print("   => Ok")
            print("   Using %s for %s" % (str(self.settings[aSetting]),aSetting))

        print("Setup finished successfully!")
        self.saveSettings()
        
        print("Execute the post process..")
        self.postProcess()


def log(message):
    print("  \033[34m{}\033[0m".format(message))

def errlog(message):
    print("\033[91m{}\033[0m".format(message))

if __name__ == '__main__':
    setup = Setup()
    setup.entry()


