using System;
using Codice.Client.Commands;
using Newtonsoft.Json;
using SmarcGUI.MissionPlanning.Tasks;


namespace SmarcGUI.Connections
{
    //https://api-docs.waraps.org/#/agent_communication/tasks/commands

    [JsonObject(NamingStrategyType = typeof(Newtonsoft.Json.Serialization.KebabCaseNamingStrategy))]
    public class BaseCommand
    {
        public string Command;
        public string ComUuid;
        public string Sender = "UnityGUI";

        public string ToJson()
        {
            return JsonConvert.SerializeObject(this);
        }

        public BaseCommand() { }

        public BaseCommand(string jsonString)
        {
            JsonConvert.PopulateObject(jsonString, this);
        }
    }

    public class PingCommand : BaseCommand
    {
        public long TimeStamp;
        public PingCommand()
        {
            Command = "ping";
            ComUuid = Guid.NewGuid().ToString();
            TimeStamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        }

        public PingCommand(string jsonString)
        {
            JsonConvert.PopulateObject(jsonString, this);
        }
    }


    public static class WaspSignals
    {
        public static string ENOUGH = "$enough";
        public static string PAUSE = "$pause";
        public static string CONTINUE = "$continue";
        public static string ABORT = "$abort";
    }

    public static class SmarcSignals
    {
        public static string CANCEL_ABORT = "$cancel_abort";
    }

    public class SigntalTaskCommand : BaseCommand
    {
        public string Signal;
        public string TaskUuid;

        public SigntalTaskCommand(string signal, string taskUuid)
        {
            Command = "signal-task";
            ComUuid = Guid.NewGuid().ToString();

            Signal = signal;
            TaskUuid = taskUuid;
        }

    }

    public class StartTaskCommand : BaseCommand
    {
        public string ExecutionUnit;
        public Task Task;
        public string TaskUuid;

        public StartTaskCommand(Task task, string robot_name)
        {
            Command = "start-task";
            ComUuid = Guid.NewGuid().ToString();

            ExecutionUnit = robot_name;
            TaskUuid = task.TaskUuid;
            Task = task;
        }
    }


    // Defined but no docs on this, so ignoring for now.
    // public class QueryStatusCommand : Command
    // {

    // }

    public class StartTSTCommand : BaseCommand
    {
        public string Receiver;
        public TaskSpecTree tst;


        public StartTSTCommand(TaskSpecTree tst, string robot_name)
        {
            //https://api-docs.waraps.org/#/agent_communication/tst/tst_commands/start_tst
            ComUuid = Guid.NewGuid().ToString();
            Command = "start-tst";
            Receiver = robot_name;

            this.tst = tst;
            this.tst.CommonParams["execunit"] = $"/{robot_name}";
            this.tst.CommonParams["node-uuid"] = Guid.NewGuid().ToString();
        }
    }
    
    public class SignalTSTUnit : BaseCommand
    {
        public string Receiver;
        public string Signal;
        public string Unit;

        public SignalTSTUnit(string signal, string robot_name)
        {
            //https://api-docs.waraps.org/#/agent_communication/tst/tst_commands/signal_tst
            ComUuid = Guid.NewGuid().ToString();
            Command = "signal-unit";
            Receiver = robot_name;

            Signal = signal;
            Unit = $"/{robot_name}";
        }
    }


}