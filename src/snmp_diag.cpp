#include <net-snmp/net-snmp-config.h>
#include <net-snmp/net-snmp-includes.h>

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>

struct snmp_session session, *ss;
std::string host = "localhost";
std::string root_oid = "1.3.6.1.2.1";
double polling_period = 1.0;
int max_items = 1000;

ros::Publisher diag_pub;

void timerCallback(const ros::TimerEvent event)
{

  diagnostic_msgs::DiagnosticArray da;
  da.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus ds;

  ds.name = "snmp";
  ds.hardware_id = host;
  ds.level = diagnostic_msgs::DiagnosticStatus::WARN;

  oid name[MAX_OID_LEN];
  size_t name_length = MAX_OID_LEN;

  if (snmp_parse_oid(root_oid.c_str(), name, &name_length) == NULL)
  {
    snmp_perror(root_oid.c_str());
    exit(2);
  }

  int item_count = 0;
  bool done = false;
  while(!done)
  {
    netsnmp_pdu* pdu = snmp_pdu_create(SNMP_MSG_GETNEXT);
    snmp_add_null_var(pdu, name, name_length);
    
    netsnmp_pdu* response = nullptr;
    int status = snmp_synch_response(ss, pdu, &response);
    if (status == STAT_SUCCESS && response && response->errstat == SNMP_ERR_NOERROR)
    {
      for(variable_list* vars = response->variables; vars; vars = vars->next_variable)
      {
        u_char* buf = nullptr;
        size_t buf_len = 0;
        size_t out_len = 0;
        int buf_overflow = 0;

        diagnostic_msgs::KeyValue kv;
        tree* subtree = netsnmp_sprint_realloc_objid_tree(&buf, &buf_len, &out_len, 1, &buf_overflow, vars->name, vars->name_length);

        if(out_len > 0)
          kv.key = reinterpret_cast<char*>(buf);
        else
          kv.key = "error";
        if(buf)
          SNMP_FREE(buf);

        buf = nullptr;
        buf_len = 0;
        out_len = 0;
        buf_overflow = 0;

        sprint_realloc_value(&buf, &buf_len, &out_len, 1, vars->name, vars->name_length, vars);
        //sprint_realloc_by_type(&buf, &buf_len, &out_len, 1, vars, subtree->enums, nullptr, nullptr);
        if(out_len > 0)
          kv.value = reinterpret_cast<char*>(buf);
        else
          kv.value = "error";
        if(buf)
          SNMP_FREE(buf);

        ds.values.push_back(kv);
        ds.level = diagnostic_msgs::DiagnosticStatus::OK;

        item_count++;
        if(item_count > max_items)
        {
          ROS_WARN("max item count reached");
          done = true;
        }

        if ((vars->type != SNMP_ENDOFMIBVIEW) && (vars->type != SNMP_NOSUCHOBJECT) && (vars->type != SNMP_NOSUCHINSTANCE))
        {
          memmove((char *) name, (char *) vars->name, vars->name_length * sizeof(oid));
          name_length = vars->name_length;
        }
        else
        {
          done = true;
        }
          
      }
    }
    else
      done = true;
    if (response)
      snmp_free_pdu(response);
  }
  da.status.push_back(ds);
  diag_pub.publish(da);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "snmp_diag");
  ros::NodeHandle nh;

  host = ros::param::param<std::string>("~host", host);
  root_oid = ros::param::param<std::string>("~root_oid", root_oid);
  polling_period = ros::param::param("~polling_period", polling_period);
  max_items = ros::param::param("~max_items", max_items);

  init_snmp("snmp_diag");
  snmp_sess_init( &session ); 
  session.peername = strdup(host.c_str());

  session.version = SNMP_VERSION_2c;
  std::string community("public");
  session.community = reinterpret_cast<u_char*>(strdup(community.c_str()));
  session.community_len = community.size();
  
  ss = snmp_open(&session); 
  if (!ss) {
    snmp_perror("ack");
    snmp_log(LOG_ERR, "something horrible happened!!!\n");
    exit(2);
  }

  netsnmp_ds_set_boolean(NETSNMP_DS_LIBRARY_ID, NETSNMP_DS_LIB_QUICK_PRINT, 1);

  ros::Timer timer = nh.createTimer(ros::Duration(polling_period), timerCallback);

  diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);

  ros::spin();
  snmp_close(ss);
  return 0;
}
