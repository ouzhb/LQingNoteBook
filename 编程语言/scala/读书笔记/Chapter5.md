- Scala原文件可以包含多个类，所有类都具有公有可见性
- 当定义了一个字段时，默认生成getter和setter方法
    - getter叫做“字段名”，setter叫做“字段名_=”
    - 通过上面的方式，我们可以重新定义getter和setter方法
    - 将字段声明为private[this]就不会生成getter、setter方法
    - val声明的字段只有getter
- private[this] 对象私有字段（方法只能访问该对象的字段），默认场景下为类私有字段（方法可以访问该类所有对象的字段）。private[Class_Name]可以对允许访问的类赋权
    - private val balance: 伴生对象可以访问
    - private[this] val balance: 伴生对象无法访问
    
- 使用@BeanProperty修饰字段时，额外生成get/set方法
    - private 字段无法用@BeanProperty修饰
- 构造器
    - 主构造器
        - 默认场景下主构造器无参数
        - 入参自动成为字段（私有的）
        - 私有主构造器 class Person private（val id：Int）{...}
    - 辅助构造器（this声明，并从另一个构造器的调用开始）
- 嵌套类
    - 同一个类的不同实例拥有不同的内部类对象
    - 使用类型投影可以避免上述情况[Network#Member]
  ```scala
  class NetWork{
  // 内部类型
  class Member(val name:String)
  
  private val member=new ArrayBuffer[Member]()
  
  def add(name:String): Unit ={
    val m= new Member(name)
    member+=m
    m
  }
}
object NetWork{
  // chatter 和 myFace都有各自的Member类型，并且是两个不一样的类型
  val chatter  =new NetWork
  val myFace=new NetWork
}
  ```