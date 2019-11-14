# Dataflow编程模型
## API抽象层次

Flink提供的API层次由高到低，如下：
- SQL：SQL操作需要在Table定义上执行
- Table API（说明性DSL）：可以在表和DataStream / DataSet之间无缝转换，允许程序混合Table API以及DataStream和DataSet API。
- DataStream/DataSet API(Core APIs)：绝大数APP，通过Core APIs来构建业务逻辑。DataStream API内部嵌入了ProcessFunction，有良好的性能。
- Stateful Stream Processing：由一系列的ProcessFunction 组成；

## 数据流

Flink中每一个Application，由以下内容组成：

- streams：数据流
- transformations：输入/输出是一个或者多个流
- sources：数据流起点 
- sinks：数据流终点

数据流有特质：

- 并行度：取决于Operator的Subtasks数目，而Flink中包括One-to-one、Redistributing 两种或类型的算子。
   
   算子的并行度可以通过以下四个方式配置：

    - Operator Level：在Stream执行算子的过程中调用setParallelism(5)
    - Execution Environment Level：在Context中决定所有算子的默认并行度
    - Client Level：提交flink任务是时指定-p参数
    - System Level：配置文件中指定parallelism.default 
  
  Flink支持配置Operator的最大并行度，默认情况下最大并行度为：max（min（32768，1.5*指定并行度），127  ）

- 窗口：Flink支持时间窗口、数据窗口，以及滑动、反转等类型的窗口操作

    - 生命周期：第一个元素到达 --> 窗口的时间戳+用户指定的允许时间延时（所以五分钟窗口的实际存在时间可能小于、大于5min）
    - Trigger：指定窗口被认为准备好执行Function的条件。同时可以清除窗口中的任意一个函数。当Trigger的某个接口返回FIRE或者FIRE_AND_PURGE时，Flink启动Function计算。
    - Function：用来计算窗口数据的函数
    - Keyed: Stream 中的每个元素可以使用keyBy()映射成键值对，键值对中Key由用户指定，value是元素本身。此时，key相同的元素在同一个subtask中运行，而对于非Keyed window任务的并行度永远是1。可以使用Field 、元组、函数来指定key。
    - Assigners：反转窗口、滑动窗口、Session窗口、全局窗口
    - Evictors：配合triger和Assigners，可以驱逐窗口中的元素
    - Event Time 和 Allowed Lateness


## Time

Flink任务中，有以下三种概念的时间：

- Event Time：数据的实际产生时间，通常 Event Time 是每天记录中的一个时间戳。
- Ingestion time：数据进入Dataflow的时间
- Processing Time：数据的实际处理时间

当一个流处理 Event Time 标记的数据时，需要一个能够衡量 Event Time 进度的工具。在Flink中，使用WaterMarks机制来衡量流的 Event Time 进度。

Watermark(t) 是一个随数据流不断变动的时间戳，表示在这个数据流中，Event Time小于t的数据已经全部到达了！当WaterMark的时间达到一定值（Window time？）时，此时会触发Stream的相关计算。

通常情况下WaterMark有以下特点：

- 在Source Functions处生成
- 每个SubTask有独立的WaterMark
- operators 有多个输入流时，WaterMark取最小值
- 对于长时间没有数据的流，可以配置assigners定期更新水印时间为处理时间。

## Stateful Operations

还没看

## Checkpoint

还没看

## Batch

还没看

# 参考

[参考](https://ci.apache.org/projects/flink/flink-docs-release-1.6/concepts/programming-model.html)

## Sink和Source

[streaming connectors](https://ci.apache.org/projects/flink/flink-docs-release-1.6/dev/connectors/index.html)

[batch connectors](https://ci.apache.org/projects/flink/flink-docs-release-1.6/dev/batch/connectors.html) 

## Stream和Batch操作

[DataStream Operators](https://ci.apache.org/projects/flink/flink-docs-release-1.6/dev/stream/operators/index.html)

[DataSet Transformations](https://ci.apache.org/projects/flink/flink-docs-release-1.6/dev/batch/dataset_transformations.html)

## API

[ DataStream API ](https://ci.apache.org/projects/flink/flink-docs-release-1.8/dev/datastream_api.html)

[ DataSet API ](https://ci.apache.org/projects/flink/flink-docs-release-1.8/dev/batch/index.html)

[ Table API ](https://ci.apache.org/projects/flink/flink-docs-release-1.8/dev/table_api.html)

[ Process Function ](https://ci.apache.org/projects/flink/flink-docs-release-1.8/concepts/programming-model.html)

[ SQL ](https://ci.apache.org/projects/flink/flink-docs-release-1.8/dev/table_api.html#sql)


## 概念说明

[Windows](https://ci.apache.org/projects/flink/flink-docs-release-1.6/dev/stream/operators/windows.html)

[Times](https://ci.apache.org/projects/flink/flink-docs-release-1.8/dev/event_time.html)

